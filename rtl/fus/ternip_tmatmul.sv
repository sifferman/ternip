// Copyright (c) 2026 Ethan Sifferman
//
// Redistribution and use in source and binary forms, with or without modification, are permitted
// provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this list of
//    conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of
//    conditions and the following disclaimer in the documentation and/or other materials provided
//    with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific prior written
//    permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
// OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// ternip_tmatmul
//
// Functional unit for ternary matrix/vector multiplication.
//
// Ternary matmul uses three required commands:
//
// IMPORT loads the input vector into the internal importvector.
// GO streams the ternary matrix from DDR, multiplies it by importvector, and
// stores the result in the internal exportvector.
// EXPORT writes exportvector to the selected vector register.
//
// The unit accepts one command on in_* and then drives the vector-register and
// DDR read interfaces until that command finishes. Use the commands in order:
// IMPORT, then GO, then EXPORT.

`define SAFE_CLOG2(x) ( (((x)==1) || ((x)==0))? 1 : $clog2((x)))

module ternip_tmatmul #(
    parameter int D                     = ternip_pkg::D,
    parameter int TmatmulParallelism    = ternip_pkg::TmatmulParallelism,
    parameter int FixedPointPrecision   = ternip_pkg::FixedPointPrecision,
    parameter int VectorParallelism     = ternip_pkg::VectorParallelism,
    parameter int NumVectorRegisters    = ternip_pkg::NumVectorRegisters,
    parameter int NumChunksPerVector    = ternip_pkg::NumChunksPerVector,
    parameter int DdrAddressWidth       = ternip_pkg::DdrAddressWidth,
    parameter int MatrixSizeInBytes     = ternip_pkg::MatrixSizeInBytes,
    parameter int NumDdrBanksPerTmatmul = ternip_pkg::NumDdrBanksPerTmatmul,

    localparam type fixed_point_t       = logic signed [ternip_pkg::FixedPointPrecision-1:0],
    localparam type vector_chunk_t      = fixed_point_t [VectorParallelism-1:0],
    localparam type ternary_t           = logic signed [1:0],
    localparam type vector_offset_t     = logic [$clog2(NumChunksPerVector)-1:0],
    localparam type vector_select_t     = logic [$clog2(NumVectorRegisters)-1:0],
    localparam type ddr_address_t       = logic [DdrAddressWidth-1:0]
) (
    input logic                     clk_i,
    input logic                     rst_ni,

    output logic                                     in_ready_o,
    input  logic                                     in_valid_i,
    input  ternip_pkg::tmatmul_op_e                  in_operation_i,
    input  vector_select_t                           in_vector_select_i,
    // GO carries one DDR address per bank. Each lane reads its own bank's
    // matrix slice from `in_go_matrix_address_i[bank]`. For single-bank
    // configs N=1 this is just a single 64-bit address.
    input  ddr_address_t [NumDdrBanksPerTmatmul-1:0] in_go_matrix_address_i,

    // vector request interface
    input  logic                    vector_request_ready_i,
    output logic                    vector_request_valid_o,
    output logic                    vector_request_write_not_read_o,
    output vector_select_t          vector_request_vector_select_o,
    output vector_offset_t          vector_request_vector_addr_o,
    output vector_chunk_t           vector_request_w_data_o,

    // vector read interface
    output logic                    vector_read_ready_o,
    input  logic                    vector_read_valid_i,
    input  vector_offset_t          vector_read_addr_i,
    input  vector_chunk_t           vector_read_data_i,

    // ddr stream start config — one set per bank
    input  logic         [NumDdrBanksPerTmatmul-1:0] ddr_stream_ready_i,
    output logic         [NumDdrBanksPerTmatmul-1:0] ddr_stream_valid_o,
    output ddr_address_t [NumDdrBanksPerTmatmul-1:0] ddr_stream_address_o,
    output logic [NumDdrBanksPerTmatmul-1:0][31:0]   ddr_stream_length_o,

    // read data streamed in sequentially — one stream per bank
    output logic     [NumDdrBanksPerTmatmul-1:0]                              ddr_r_ready_o,
    input  logic     [NumDdrBanksPerTmatmul-1:0]                              ddr_r_valid_i,
    input  ternary_t [NumDdrBanksPerTmatmul-1:0][TmatmulParallelism-1:0]      ddr_r_data_i
);

localparam int RowParallelism = (TmatmulParallelism < D) ? (1) : (TmatmulParallelism / D);
localparam int DdrReadsPerRow = (TmatmulParallelism > D) ? (1) : (D / TmatmulParallelism);
localparam int ImportVectorRowWidth = ternip_pkg::min_int(TmatmulParallelism, D);

// Per-bank derived sizes. Each bank owns a contiguous block of D/N rows.
localparam int RowsPerBank           = D / NumDdrBanksPerTmatmul;
localparam int MatrixBytesPerBank    = MatrixSizeInBytes / NumDdrBanksPerTmatmul;
localparam int ChunksPerBankExport   = RowsPerBank / VectorParallelism;

`ifndef SYNTHESIS
initial begin
    if (TmatmulParallelism < D) begin
        assert(DdrReadsPerRow * TmatmulParallelism == D) else begin
            $fatal("TmatmulParallelism (%0d) must divide evenly into D (%0d)", TmatmulParallelism, D);
        end
    end else begin
        assert(RowParallelism * D == TmatmulParallelism) else begin
            $fatal("D (%0d) must divide evenly into TmatmulParallelism (%0d)", D, TmatmulParallelism);
        end
    end
end
`endif

typedef logic signed [FixedPointPrecision:0] tmul_result_t;

function automatic tmul_result_t ternary_mul(ternary_t ternary, fixed_point_t fixed_point);
    unique case (ternary)
        0: return '0;
        1: return fixed_point;
        -1: return -fixed_point;
        default: return 'x;
    endcase
endfunction

enum logic [1:0] {
    WAITING_FOR_IN,
    WAITING_FOR_DDR_STREAM_READY,
    WORKING
} state_d, state_q = WAITING_FOR_IN; // for assertions at time=0

ternip_pkg::tmatmul_op_e tmatmul_operation_d, tmatmul_operation_q;
vector_select_t vector_select_d, vector_select_q;

localparam int DdrReadsPerMatrix = (D*D) / TmatmulParallelism;

// Each bank streams MatrixBytesPerBank = MatrixSizeInBytes / N bytes.
for (genvar b_GEN = 0; b_GEN < NumDdrBanksPerTmatmul; b_GEN++) begin : ddr_stream_length_assign
    assign ddr_stream_length_o[b_GEN] = MatrixBytesPerBank;
end

logic [`SAFE_CLOG2(DdrReadsPerMatrix):0] importvector_counter_d, importvector_counter_q;
logic [`SAFE_CLOG2(DdrReadsPerMatrix):0] exportvector_counter_d, exportvector_counter_q;
logic [NumDdrBanksPerTmatmul-1:0][`SAFE_CLOG2(ChunksPerBankExport):0] go_exportvector_counter_d, go_exportvector_counter_q;
logic go_export_done;
// One latched start-of-stream address per bank. The kernel hands GO a full
// per-bank address vector (`in_go_matrix_address_i[bank]`); we hold it here
// across the streaming op so the value survives any handshake stalls.
ddr_address_t [NumDdrBanksPerTmatmul-1:0] ddr_stream_address_d, ddr_stream_address_q;
logic         [NumDdrBanksPerTmatmul-1:0] ddr_stream_accepted_d, ddr_stream_accepted_q;
logic [`SAFE_CLOG2(DdrReadsPerRow):0] importvector_request_addr_counter_d, importvector_request_addr_counter_q;
logic     [NumDdrBanksPerTmatmul-1:0]                         go_ddr_valid_d, go_ddr_valid_q;
ternary_t [NumDdrBanksPerTmatmul-1:0][TmatmulParallelism-1:0] go_ddr_data_d, go_ddr_data_q;

// AXI stream adapter for IMPORT: vector_chunk_t -> fixed_point_t (scalar — IMPORT is shared)
logic gbfifo_import_in_ready;
logic gbfifo_import_in_valid;
vector_chunk_t gbfifo_import_in_data;

logic gbfifo_import_out_ready;
logic gbfifo_import_out_valid;
fixed_point_t [ImportVectorRowWidth-1:0] gbfifo_import_out_data;

// AXI stream adapters for GO (export): fixed_point_t -> vector_chunk_t. One per bank.
logic         [NumDdrBanksPerTmatmul-1:0]                    gbfifo_export_in_ready;
logic         [NumDdrBanksPerTmatmul-1:0]                    gbfifo_export_in_valid;
fixed_point_t [NumDdrBanksPerTmatmul-1:0][RowParallelism-1:0] gbfifo_export_in_data;

logic          [NumDdrBanksPerTmatmul-1:0] gbfifo_export_out_ready;
logic          [NumDdrBanksPerTmatmul-1:0] gbfifo_export_out_valid;
vector_chunk_t [NumDdrBanksPerTmatmul-1:0] gbfifo_export_out_data;

// IMPORT BRAM write tracking
logic [`SAFE_CLOG2(DdrReadsPerRow):0] import_bram_addr_d, import_bram_addr_q;

// EXPORT state: walk N banks sequentially. Request and response sides each
// track their own (bank, local) pair to avoid expensive div/mod on the
// response side when ChunksPerBankExport isn't a power of 2.
logic [`SAFE_CLOG2(NumDdrBanksPerTmatmul):0]    export_bank_request_d,   export_bank_request_q;
logic [`SAFE_CLOG2(ChunksPerBankExport):0]      export_local_request_d,  export_local_request_q;
logic [`SAFE_CLOG2(NumDdrBanksPerTmatmul):0]    export_bank_response_d,  export_bank_response_q;
logic [`SAFE_CLOG2(ChunksPerBankExport):0]      export_local_response_d, export_local_response_q;

// Queued Instruction
logic queued_valid_d, queued_valid_q;
ternip_pkg::tmatmul_op_e queued_operation_d, queued_operation_q;
vector_select_t queued_vector_select_d, queued_vector_select_q;
ddr_address_t [NumDdrBanksPerTmatmul-1:0] queued_go_matrix_address_d, queued_go_matrix_address_q;

// Multioperand Accumulator — one TP-wide MOA per bank. Lanes run in lockstep:
// accumulator_in_valid and accumulator_in_final are broadcast, while operands
// and ready/result are per-lane.
logic accumulator_in_valid;
logic accumulator_in_final;
tmul_result_t [NumDdrBanksPerTmatmul-1:0][RowParallelism-1:0][ImportVectorRowWidth-1:0] accumulator_operands;
logic         [NumDdrBanksPerTmatmul-1:0][RowParallelism-1:0]                            accumulator_in_ready;
logic         [NumDdrBanksPerTmatmul-1:0][RowParallelism-1:0]                            accumulator_out_valid;
fixed_point_t [NumDdrBanksPerTmatmul-1:0][RowParallelism-1:0]                            accumulator_result;
// Per-bank accumulator_out_ready; each bank's gbfifo_export drives its own
logic         [NumDdrBanksPerTmatmul-1:0]                                                accumulator_out_ready;

// Per-bank exportvector signals (instances are inside the bank_lane generate below).
// Declared up here so the generate-loop port-map references resolve forward.
logic          [NumDdrBanksPerTmatmul-1:0]                                       exportvector_request_ready;
logic          [NumDdrBanksPerTmatmul-1:0]                                       exportvector_request_valid;
logic          [NumDdrBanksPerTmatmul-1:0]                                       exportvector_request_write_not_read;
logic          [NumDdrBanksPerTmatmul-1:0][`SAFE_CLOG2(ChunksPerBankExport)-1:0] exportvector_request_addr;
vector_chunk_t [NumDdrBanksPerTmatmul-1:0]                                       exportvector_request_w_data;

logic          [NumDdrBanksPerTmatmul-1:0]                                       exportvector_read_ready;
logic          [NumDdrBanksPerTmatmul-1:0]                                       exportvector_read_valid;
logic          [NumDdrBanksPerTmatmul-1:0][`SAFE_CLOG2(ChunksPerBankExport)-1:0] exportvector_read_addr;
vector_chunk_t [NumDdrBanksPerTmatmul-1:0]                                       exportvector_read_data;

for (genvar bank_GEN = 0; bank_GEN < NumDdrBanksPerTmatmul; bank_GEN++) begin : bank_lane

    // Per-lane MOA
    for (genvar i_GEN = 0; i_GEN < RowParallelism; i_GEN++) begin : row
        ternip_multioperand_accumulator #(
            .operand_t(tmul_result_t),
            .result_t(fixed_point_t),
            .NUM_OPERANDS(ImportVectorRowWidth),
            .NEXT_STAGE_FANIN(4)
        ) multioperand_accumulator (
            .clk_i,
            .rst_ni,
            .in_ready_o(accumulator_in_ready[bank_GEN][i_GEN]),
            .in_valid_i(accumulator_in_valid),
            .in_final_i(accumulator_in_final),
            .in_operands_i(accumulator_operands[bank_GEN][i_GEN]),
            .out_ready_i(accumulator_out_ready[bank_GEN]),
            .out_valid_o(accumulator_out_valid[bank_GEN][i_GEN]),
            .out_result_o(accumulator_result[bank_GEN][i_GEN])
        );
    end

    // Per-lane EXPORT gearbox_fifo: fixed_point_t -> vector_chunk_t
    ternip_gearbox_fifo #(
        .InDataWidth($bits(gbfifo_export_in_data[bank_GEN])),
        .OutDataWidth($bits(gbfifo_export_out_data[bank_GEN]))
    ) gbfifo_export (
        .clk_i,
        .rst_ni,

        .in_ready_o(gbfifo_export_in_ready[bank_GEN]),
        .in_valid_i(gbfifo_export_in_valid[bank_GEN]),
        .in_data_i(gbfifo_export_in_data[bank_GEN]),

        .out_ready_i(gbfifo_export_out_ready[bank_GEN]),
        .out_valid_o(gbfifo_export_out_valid[bank_GEN]),
        .out_data_o(gbfifo_export_out_data[bank_GEN])
    );

    // Per-lane exportvector. Sized to one bank's row chunks
    // (ChunksPerBankExport = RowsPerBank / VectorParallelism). The full vector
    // is reassembled in EXPORT by walking through banks sequentially.
    ternip_pipelined_mem #(
        .DATA_WIDTH($bits(exportvector_read_data[bank_GEN])),
        .NUM_ENTRIES(ChunksPerBankExport)
    ) exportvector (
        .clk_i,
        .rst_ni,

        .request_ready_o(exportvector_request_ready[bank_GEN]),
        .request_valid_i(exportvector_request_valid[bank_GEN]),
        .request_write_not_read_i(exportvector_request_write_not_read[bank_GEN]),
        .request_addr_i(exportvector_request_addr[bank_GEN]),
        .request_w_data_i(exportvector_request_w_data[bank_GEN]),

        .read_ready_i(exportvector_read_ready[bank_GEN]),
        .read_valid_o(exportvector_read_valid[bank_GEN]),
        .read_addr_o(exportvector_read_addr[bank_GEN]),
        .read_data_o(exportvector_read_data[bank_GEN])
    );

end

// Import Vector
logic importvector_request_ready;
logic importvector_request_valid;
logic importvector_request_write_not_read;
logic [`SAFE_CLOG2(DdrReadsPerRow)-1:0] importvector_request_addr;
fixed_point_t [ImportVectorRowWidth-1:0] importvector_request_w_data;

logic importvector_read_ready;
fixed_point_t [ImportVectorRowWidth-1:0] importvector_read_data;
logic importvector_read_valid;
logic [`SAFE_CLOG2(DdrReadsPerRow)-1:0] importvector_read_addr;

ternip_pipelined_mem #(
    .DATA_WIDTH($bits(importvector_read_data)),
    .NUM_ENTRIES(DdrReadsPerRow)
) importvector (
    .clk_i,
    .rst_ni,

    .request_ready_o(importvector_request_ready),
    .request_valid_i(importvector_request_valid),
    .request_write_not_read_i(importvector_request_write_not_read),
    .request_addr_i(importvector_request_addr),
    .request_w_data_i(importvector_request_w_data),

    .read_ready_i(importvector_read_ready),
    .read_valid_o(importvector_read_valid),
    .read_addr_o(importvector_read_addr),
    .read_data_o(importvector_read_data)
);

// IMPORT gearbox_fifo: vector_chunk_t -> fixed_point_t [ImportVectorRowWidth-1:0] (scalar — shared)
ternip_gearbox_fifo #(
    .InDataWidth($bits(gbfifo_import_in_data)),
    .OutDataWidth($bits(gbfifo_import_out_data))
) gbfifo_import (
    .clk_i,
    .rst_ni,

    .in_ready_o(gbfifo_import_in_ready),
    .in_valid_i(gbfifo_import_in_valid),
    .in_data_i(gbfifo_import_in_data),

    .out_ready_i(gbfifo_import_out_ready),
    .out_valid_o(gbfifo_import_out_valid),
    .out_data_o(gbfifo_import_out_data)
);

// Reduction helper: AND of all per-lane MOA in_ready bits.
logic all_moas_in_ready;
always_comb begin
    all_moas_in_ready = 1'b1;
    for (int b = 0; b < NumDdrBanksPerTmatmul; b++) begin
        for (int r = 0; r < RowParallelism; r++) begin
            all_moas_in_ready &= accumulator_in_ready[b][r];
        end
    end
end
logic go_input_fire;

always_comb begin
    state_d = state_q;
    in_ready_o = 0;

    queued_valid_d = queued_valid_q;
    queued_operation_d = queued_operation_q;
    queued_vector_select_d = queued_vector_select_q;
    queued_go_matrix_address_d = queued_go_matrix_address_q;

    tmatmul_operation_d = tmatmul_operation_q;
    exportvector_counter_d = exportvector_counter_q;
    go_exportvector_counter_d = go_exportvector_counter_q;
    ddr_stream_address_d = ddr_stream_address_q;
    ddr_stream_accepted_d = ddr_stream_accepted_q;
    importvector_request_addr_counter_d = importvector_request_addr_counter_q;
    go_ddr_valid_d = go_ddr_valid_q;
    go_ddr_data_d = go_ddr_data_q;

    importvector_counter_d = importvector_counter_q;
    vector_select_d = vector_select_q;
    import_bram_addr_d = import_bram_addr_q;

    export_bank_request_d    = export_bank_request_q;
    export_local_request_d   = export_local_request_q;
    export_bank_response_d   = export_bank_response_q;
    export_local_response_d  = export_local_response_q;

    vector_request_valid_o = 0;
    vector_request_write_not_read_o = 'x;
    vector_request_vector_select_o = 'x;
    vector_request_vector_addr_o = 'x;
    vector_request_w_data_o = 'x;
    vector_read_ready_o = 0;

    ddr_stream_address_o = '{default: 'x};
    ddr_stream_valid_o = '0;
    ddr_r_ready_o = '0;

    accumulator_in_valid = 0;
    accumulator_in_final = 0;
    accumulator_operands = '{default: 'x};
    accumulator_out_ready = '0;

    importvector_request_valid = '0;
    importvector_request_write_not_read = 'x;
    importvector_request_addr = 'x;
    importvector_request_w_data = 'x;
    importvector_read_ready = '0;

    exportvector_request_valid          = '0;
    exportvector_request_write_not_read = '{default: 'x};
    exportvector_request_addr           = '{default: 'x};
    exportvector_request_w_data         = '{default: 'x};
    exportvector_read_ready             = '0;

    gbfifo_import_in_valid = 0;
    gbfifo_import_in_data = 'x;
    gbfifo_import_out_ready = 0;

    gbfifo_export_in_valid = '0;
    gbfifo_export_in_data = '{default: 'x};
    gbfifo_export_out_ready = '0;
    go_export_done = 0;
    go_input_fire = 0;

    if (state_q == WAITING_FOR_IN) begin

        in_ready_o = !queued_valid_q;

        importvector_counter_d = 0;
        exportvector_counter_d = 0;
        go_exportvector_counter_d = '0;
        importvector_request_addr_counter_d = 0;
        ddr_stream_accepted_d = '0;
        go_ddr_valid_d = '0;
        go_ddr_data_d = '{default: 'x};
        import_bram_addr_d = 0;
        export_bank_request_d    = '0;
        export_local_request_d   = '0;
        export_bank_response_d   = '0;
        export_local_response_d  = '0;

        if (queued_valid_q) begin
            queued_valid_d = 0;
            queued_operation_d = ternip_pkg::NO_TMATMUL_OP;
            queued_vector_select_d = 'x;
            queued_go_matrix_address_d = 'x;

            tmatmul_operation_d = queued_operation_q;
            vector_select_d = queued_vector_select_q;
            ddr_stream_address_d = queued_go_matrix_address_q;

            state_d = (tmatmul_operation_d == ternip_pkg::GO) ? (WAITING_FOR_DDR_STREAM_READY) : (WORKING);

        end else if (in_valid_i) begin
            tmatmul_operation_d = in_operation_i;
            vector_select_d = in_vector_select_i;
            ddr_stream_address_d = in_go_matrix_address_i;

            state_d = (tmatmul_operation_d == ternip_pkg::GO) ? (WAITING_FOR_DDR_STREAM_READY) : (WORKING);
        end

    end else if ((state_q == WAITING_FOR_DDR_STREAM_READY) && (tmatmul_operation_q == ternip_pkg::GO)) begin

        // Each bank streams its own slice from its own DDR bank. The host
        // pre-computes one address per bank (accounting for whatever physical
        // layout the platform / testbench uses) and the kernel just drives
        // each lane with its bank-specific value.
        ddr_stream_valid_o = ~ddr_stream_accepted_q;
        ddr_stream_address_o = ddr_stream_address_q;
        ddr_stream_accepted_d = ddr_stream_accepted_q | (ddr_stream_valid_o & ddr_stream_ready_i);
        if (&ddr_stream_accepted_d) begin
            state_d = WORKING;
        end

    end else if ((state_q == WORKING) && (tmatmul_operation_q == ternip_pkg::IMPORT)) begin

        // Request chunks from vector registers
        if (importvector_counter_q < NumChunksPerVector) begin
            vector_request_valid_o = 1;
            vector_request_write_not_read_o = 0;
            vector_request_vector_select_o = vector_select_q;
            vector_request_vector_addr_o = importvector_counter_q;
            if (vector_request_ready_i) importvector_counter_d++;
        end

        // Feed vector chunks into adapter
        vector_read_ready_o = gbfifo_import_in_ready;
        gbfifo_import_in_valid = vector_read_valid_i;
        gbfifo_import_in_data = vector_read_data_i;

        // Receive deserialized elements from adapter and write to BRAMs
        gbfifo_import_out_ready = importvector_request_ready;
        if (gbfifo_import_out_ready && gbfifo_import_out_valid) begin
            importvector_request_valid = 1;
            importvector_request_write_not_read = 1;
            importvector_request_addr = import_bram_addr_q;
            importvector_request_w_data = gbfifo_import_out_data;

            import_bram_addr_d++;
            if (import_bram_addr_q >= DdrReadsPerRow-1) begin
                state_d = WAITING_FOR_IN;
                tmatmul_operation_d = ternip_pkg::NO_TMATMUL_OP;
            end
        end

    end else if ((state_q == WORKING) && (tmatmul_operation_q == ternip_pkg::GO)) begin

        // allow instruction queueing
        in_ready_o = !queued_valid_q;
        if (in_ready_o && in_valid_i) begin
            queued_operation_d = in_operation_i;
            queued_go_matrix_address_d = in_go_matrix_address_i;
            queued_vector_select_d = in_vector_select_i;
            queued_valid_d = 1;
        end

        // Request reads from the shared importvector. All N lanes are in
        // lockstep on column-chunk position within their respective rows, so
        // a single broadcast read per beat suffices. Total beats per bank =
        // D²/(N*TP); per-bank counter is implicit (lanes consume in step).
        if (importvector_counter_q < DdrReadsPerMatrix / NumDdrBanksPerTmatmul) begin
            importvector_request_valid = '1;
            importvector_request_write_not_read = '0;
            importvector_request_addr = importvector_request_addr_counter_q;
            if (importvector_request_ready) begin
                importvector_counter_d++;
                importvector_request_addr_counter_d++;
                if (importvector_request_addr_counter_q == DdrReadsPerRow-1)
                    importvector_request_addr_counter_d = 0;
            end
        end

        // Capture one DDR beat per bank, then rendezvous all captured beats
        // with one importvector beat before the shared accumulator input
        // advances. This lets bank streams arrive with independent bubbles.
        for (int b = 0; b < NumDdrBanksPerTmatmul; b++) begin
            ddr_r_ready_o[b] = !go_ddr_valid_q[b];
            if (ddr_r_ready_o[b] && ddr_r_valid_i[b]) begin
                go_ddr_valid_d[b] = 1;
                go_ddr_data_d[b] = ddr_r_data_i[b];
            end
        end
        go_input_fire = importvector_read_valid && all_moas_in_ready && (&go_ddr_valid_d);
        if (go_input_fire) begin
                importvector_read_ready = 1;
                accumulator_in_valid    = 1;
                for (int b = 0; b < NumDdrBanksPerTmatmul; b++) begin
                    for (int i = 0; i < TmatmulParallelism; i++) begin
                        accumulator_operands[b][i / D][i % D] = ternary_mul(
                            go_ddr_data_d[b][i], importvector_read_data[i % D]);
                    end
                end
                if (importvector_read_addr >= DdrReadsPerRow-1) begin
                    accumulator_in_final = 1;
                end
                go_ddr_valid_d = '0;
                go_ddr_data_d = '{default: 'x};
        end

        // Per-lane: MOA result -> gbfifo_export -> bank's exportvector.
        // Each bank has its own write counter because its local FIFO/memory
        // handshakes can drift from the other banks.
        for (int b = 0; b < NumDdrBanksPerTmatmul; b++) begin
            accumulator_out_ready[b]  = gbfifo_export_in_ready[b] && (go_exportvector_counter_q[b] < ChunksPerBankExport);
            gbfifo_export_in_valid[b] = accumulator_out_valid[b][0] && (go_exportvector_counter_q[b] < ChunksPerBankExport);
            gbfifo_export_in_data[b]  = accumulator_result[b];

            gbfifo_export_out_ready[b] = exportvector_request_ready[b] && (go_exportvector_counter_q[b] < ChunksPerBankExport);
            if (gbfifo_export_out_ready[b] && gbfifo_export_out_valid[b]) begin
                exportvector_request_valid[b]          = 1;
                exportvector_request_write_not_read[b] = 1;
                exportvector_request_addr[b]           = go_exportvector_counter_q[b][`SAFE_CLOG2(ChunksPerBankExport)-1:0];
                exportvector_request_w_data[b]         = gbfifo_export_out_data[b];
                go_exportvector_counter_d[b] = go_exportvector_counter_q[b] + 1;
            end
        end
        go_export_done = 1;
        for (int b = 0; b < NumDdrBanksPerTmatmul; b++) begin
            go_export_done &= (go_exportvector_counter_d[b] >= ChunksPerBankExport);
        end
        if (go_export_done) begin
            state_d = WAITING_FOR_IN;
            tmatmul_operation_d = ternip_pkg::NO_TMATMUL_OP;
            go_exportvector_counter_d = '0;
        end

    end else if ((state_q == WORKING) && (tmatmul_operation_q == ternip_pkg::EXPORT)) begin

        // EXPORT: walk the N exportvectors sequentially, draining each bank's
        // ChunksPerBankExport chunks before advancing. Request and response
        // sides track separately so they can pipeline. Bank index for each
        // side is derived from its counter / ChunksPerBankExport.

        // -- Request side --
        if (export_bank_request_q < NumDdrBanksPerTmatmul) begin
            exportvector_request_valid[export_bank_request_q]          = 1;
            exportvector_request_write_not_read[export_bank_request_q] = 0;
            exportvector_request_addr[export_bank_request_q]           = export_local_request_q[`SAFE_CLOG2(ChunksPerBankExport)-1:0];
            if (exportvector_request_ready[export_bank_request_q]) begin
                if (export_local_request_q == ChunksPerBankExport-1) begin
                    export_local_request_d = '0;
                    export_bank_request_d++;
                end else begin
                    export_local_request_d++;
                end
            end
        end

        // -- Response side --
        if (export_bank_response_q < NumDdrBanksPerTmatmul) begin : export_response_blk
            // Global chunk index in vector_register = bank*ChunksPerBankExport + local.
            // ChunksPerBankExport is a compile-time constant so the multiplier folds.
            automatic logic [`SAFE_CLOG2(NumChunksPerVector)-1:0] resp_global_idx;
            resp_global_idx = export_bank_response_q[`SAFE_CLOG2(NumDdrBanksPerTmatmul)-1:0] * ChunksPerBankExport
                              + export_local_response_q[`SAFE_CLOG2(ChunksPerBankExport)-1:0];

            exportvector_read_ready[export_bank_response_q] = vector_request_ready_i;
            if (exportvector_read_ready[export_bank_response_q] && exportvector_read_valid[export_bank_response_q]) begin
                vector_request_valid_o          = 1;
                vector_request_write_not_read_o = '1;
                vector_request_vector_select_o  = vector_select_q;
                vector_request_vector_addr_o    = resp_global_idx;
                vector_request_w_data_o         = exportvector_read_data[export_bank_response_q];

                if (export_bank_response_q == NumDdrBanksPerTmatmul-1
                        && export_local_response_q == ChunksPerBankExport-1) begin
                    state_d = WAITING_FOR_IN;
                    tmatmul_operation_d = ternip_pkg::NO_TMATMUL_OP;
                end

                if (export_local_response_q == ChunksPerBankExport-1) begin
                    export_local_response_d = '0;
                    export_bank_response_d++;
                end else begin
                    export_local_response_d++;
                end
            end
        end

    end
end

always_ff @(posedge clk_i) begin
    if (!rst_ni) begin
        queued_valid_q <= '0;
        state_q <= WAITING_FOR_IN;
        go_ddr_valid_q <= '0;
        ddr_stream_accepted_q <= '0;
    end else begin
        queued_valid_q <= queued_valid_d;
        state_q <= state_d;
        go_ddr_valid_q <= go_ddr_valid_d;
        ddr_stream_accepted_q <= ddr_stream_accepted_d;
    end
end
always_ff @(posedge clk_i) begin
    vector_select_q <= vector_select_d;
    tmatmul_operation_q <= tmatmul_operation_d;

    queued_operation_q <= queued_operation_d;
    queued_vector_select_q <= queued_vector_select_d;
    queued_go_matrix_address_q <= queued_go_matrix_address_d;

    importvector_counter_q <= importvector_counter_d;
    importvector_request_addr_counter_q <= importvector_request_addr_counter_d;
    exportvector_counter_q <= exportvector_counter_d;
    go_exportvector_counter_q <= go_exportvector_counter_d;
    import_bram_addr_q <= import_bram_addr_d;
    export_bank_request_q <= export_bank_request_d;
    export_local_request_q <= export_local_request_d;
    export_bank_response_q <= export_bank_response_d;
    export_local_response_q <= export_local_response_d;
    `ifndef SYNTHESIS
    if (!rst_ni) begin
        vector_select_q <= 'x;
        tmatmul_operation_q <= ternip_pkg::NO_TMATMUL_OP;

        queued_operation_q <= ternip_pkg::NO_TMATMUL_OP;
        queued_vector_select_q <= 'x;
        queued_go_matrix_address_q <= 'x;

        importvector_counter_q <= 'x;
        importvector_request_addr_counter_q <= 'x;
        exportvector_counter_q <= 'x;
        go_exportvector_counter_q <= 'x;
        import_bram_addr_q <= 'x;
        export_bank_request_q    <= 'x;
        export_local_request_q   <= 'x;
        export_bank_response_q   <= 'x;
        export_local_response_q  <= 'x;
    end
    `endif
end
always_ff @(posedge clk_i) begin
    go_ddr_data_q <= go_ddr_data_d;
    ddr_stream_address_q <= ddr_stream_address_d;
    `ifndef SYNTHESIS
    if (!rst_ni) begin
        ddr_stream_address_q <= 'x;
        go_ddr_data_q <= '{default: 'x};
    end
    `endif
end

endmodule

`undef SAFE_CLOG2
