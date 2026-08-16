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
    parameter ternip_pkg::ternip_cfg_t Cfg = `TERNIP_CFG
) (
    input  logic                                              clk_i,
    input  logic                                              rst_ni,

    output logic                                              in_ready_o,
    input  logic                                              in_valid_i,
    input  ternip_pkg::tmatmul_op_e                           in_operation_i,
    input  ternip_types#(Cfg)::vector_select_t                in_vector_select_i,
    input  ternip_types#(Cfg)::ddr_address_t                  in_go_matrix_address_i,

    // vector request interface
    input  logic                                              vector_request_ready_i,
    output logic                                              vector_request_valid_o,
    output logic                                              vector_request_write_not_read_o,
    output ternip_types#(Cfg)::vector_select_t                vector_request_vector_select_o,
    output ternip_types#(Cfg)::vector_offset_t                vector_request_vector_addr_o,
    output ternip_types#(Cfg)::vector_chunk_t                 vector_request_w_data_o,

    // vector read interface
    output logic                                              vector_read_ready_o,
    input  logic                                              vector_read_valid_i,
    input  ternip_types#(Cfg)::vector_offset_t                vector_read_addr_i,
    input  ternip_types#(Cfg)::vector_chunk_t                 vector_read_data_i,

    // ddr stream start config
    input  logic                                              ddr_stream_ready_i,
    output logic                                              ddr_stream_valid_o,
    output ternip_types#(Cfg)::ddr_address_t                  ddr_stream_address_o,
    output logic [31:0]                                       ddr_stream_length_o,

    // read data is streamed in sequentially
    output logic                                              ddr_r_ready_o,
    input  logic                                              ddr_r_valid_i,
    input  ternip_pkg::ternary_t [Cfg.TmatmulParallelism-1:0] ddr_r_data_i
);

localparam int RowParallelism = (Cfg.TmatmulParallelism < Cfg.D) ? (1) : (Cfg.TmatmulParallelism / Cfg.D);
localparam int DdrReadsPerRow = (Cfg.TmatmulParallelism > Cfg.D) ? (1) : (Cfg.D / Cfg.TmatmulParallelism);
localparam int ImportVectorRowWidth = ternip_pkg::min_int(Cfg.TmatmulParallelism, Cfg.D);

`ifndef SYNTHESIS
initial begin
    if (Cfg.TmatmulParallelism < Cfg.D) begin
        assert(DdrReadsPerRow * Cfg.TmatmulParallelism == Cfg.D) else begin
            $fatal("Cfg.TmatmulParallelism (%0d) must divide evenly into Cfg.D (%0d)", Cfg.TmatmulParallelism, Cfg.D);
        end
    end else begin
        assert(RowParallelism * Cfg.D == Cfg.TmatmulParallelism) else begin
            $fatal("Cfg.D (%0d) must divide evenly into Cfg.TmatmulParallelism (%0d)", Cfg.D, Cfg.TmatmulParallelism);
        end
    end
end
`endif

typedef logic signed [Cfg.FixedPointPrecision:0] tmul_result_t;

function automatic tmul_result_t ternary_mul(ternip_pkg::ternary_t ternary, ternip_types#(Cfg)::fixed_point_t fixed_point);
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
ternip_types#(Cfg)::vector_select_t vector_select_d, vector_select_q;

localparam int DdrReadsPerMatrix = (Cfg.D*Cfg.D) / Cfg.TmatmulParallelism;
assign ddr_stream_length_o = ternip_types#(Cfg)::MatrixSizeInBytes;

logic [`SAFE_CLOG2(DdrReadsPerMatrix):0] importvector_counter_d, importvector_counter_q;
logic [`SAFE_CLOG2(DdrReadsPerMatrix):0] exportvector_counter_d, exportvector_counter_q;
ternip_types#(Cfg)::ddr_address_t ddr_stream_address_d, ddr_stream_address_q;
logic [`SAFE_CLOG2(DdrReadsPerRow):0] importvector_request_addr_counter_d, importvector_request_addr_counter_q;

// AXI stream adapter for IMPORT: ternip_types#(Cfg)::vector_chunk_t -> ternip_types#(Cfg)::fixed_point_t
logic gbfifo_import_in_ready;
logic gbfifo_import_in_valid;
ternip_types#(Cfg)::vector_chunk_t gbfifo_import_in_data;

logic gbfifo_import_out_ready;
logic gbfifo_import_out_valid;
ternip_types#(Cfg)::fixed_point_t [ImportVectorRowWidth-1:0] gbfifo_import_out_data;

// AXI stream adapter for GO (export): ternip_types#(Cfg)::fixed_point_t -> ternip_types#(Cfg)::vector_chunk_t
logic gbfifo_export_in_ready;
logic gbfifo_export_in_valid;
ternip_types#(Cfg)::fixed_point_t [RowParallelism-1:0] gbfifo_export_in_data;

logic gbfifo_export_out_ready;
logic gbfifo_export_out_valid;
ternip_types#(Cfg)::vector_chunk_t gbfifo_export_out_data;

// IMPORT BRAM write tracking
logic [`SAFE_CLOG2(DdrReadsPerRow):0] import_bram_addr_d, import_bram_addr_q;

// Queued Instruction
logic queued_valid_d, queued_valid_q;
ternip_pkg::tmatmul_op_e queued_operation_d, queued_operation_q;
ternip_types#(Cfg)::vector_select_t queued_vector_select_d, queued_vector_select_q;
ternip_types#(Cfg)::ddr_address_t queued_go_matrix_address_d, queued_go_matrix_address_q;

// Multioperand Accumulator
logic accumulator_in_valid, accumulator_in_final, accumulator_out_ready;
tmul_result_t [RowParallelism-1:0][ImportVectorRowWidth-1:0] accumulator_operands;
logic [RowParallelism-1:0] accumulator_in_ready, accumulator_out_valid;
ternip_types#(Cfg)::fixed_point_t [RowParallelism-1:0] accumulator_result;

for (genvar i_GEN = 0; i_GEN < RowParallelism; i_GEN++) begin : row

    ternip_multioperand_accumulator #(
        .operand_t(tmul_result_t),
        .result_t(ternip_types#(Cfg)::fixed_point_t),
        .NUM_OPERANDS(ImportVectorRowWidth),
        .NEXT_STAGE_FANIN(2)
    ) multioperand_accumulator (
        .clk_i,
        .rst_ni,
        .in_ready_o(accumulator_in_ready[i_GEN]),
        .in_valid_i(accumulator_in_valid),
        .in_final_i(accumulator_in_final),
        .in_operands_i(accumulator_operands[i_GEN]),
        .out_ready_i(accumulator_out_ready),
        .out_valid_o(accumulator_out_valid[i_GEN]),
        .out_result_o(accumulator_result[i_GEN])
    );

end

// Import Vector
logic importvector_request_ready;
logic importvector_request_valid;
logic importvector_request_write_not_read;
logic [`SAFE_CLOG2(DdrReadsPerRow)-1:0] importvector_request_addr;
ternip_types#(Cfg)::fixed_point_t [ImportVectorRowWidth-1:0] importvector_request_w_data;

logic importvector_read_ready;
ternip_types#(Cfg)::fixed_point_t [ImportVectorRowWidth-1:0] importvector_read_data;
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

// Export Vector
logic exportvector_request_ready;
logic exportvector_request_valid;
logic exportvector_request_write_not_read;
logic [`SAFE_CLOG2(ternip_types#(Cfg)::NumChunksPerVector)-1:0] exportvector_request_addr;
ternip_types#(Cfg)::vector_chunk_t exportvector_request_w_data;

logic exportvector_read_ready;
logic exportvector_read_valid;
logic [`SAFE_CLOG2(ternip_types#(Cfg)::NumChunksPerVector)-1:0] exportvector_read_addr;
ternip_types#(Cfg)::vector_chunk_t exportvector_read_data;

ternip_pipelined_mem #(
    .DATA_WIDTH($bits(exportvector_read_data)),
    .NUM_ENTRIES(ternip_types#(Cfg)::NumChunksPerVector)
) exportvector (
    .clk_i,
    .rst_ni,

    .request_ready_o(exportvector_request_ready),
    .request_valid_i(exportvector_request_valid),
    .request_write_not_read_i(exportvector_request_write_not_read),
    .request_addr_i(exportvector_request_addr),
    .request_w_data_i(exportvector_request_w_data),

    .read_ready_i(exportvector_read_ready),
    .read_valid_o(exportvector_read_valid),
    .read_addr_o(exportvector_read_addr),
    .read_data_o(exportvector_read_data)
);

// IMPORT gearbox_fifo: ternip_types#(Cfg)::vector_chunk_t -> ternip_types#(Cfg)::fixed_point_t [Cfg.TmatmulParallelism-1:0]
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

// EXPORT (GO) gearbox_fifo: ternip_types#(Cfg)::fixed_point_t -> ternip_types#(Cfg)::vector_chunk_t
ternip_gearbox_fifo #(
    .InDataWidth($bits(gbfifo_export_in_data)),
    .OutDataWidth($bits(gbfifo_export_out_data))
) gbfifo_export (
    .clk_i,
    .rst_ni,

    .in_ready_o(gbfifo_export_in_ready),
    .in_valid_i(gbfifo_export_in_valid),
    .in_data_i(gbfifo_export_in_data),

    .out_ready_i(gbfifo_export_out_ready),
    .out_valid_o(gbfifo_export_out_valid),
    .out_data_o(gbfifo_export_out_data)
);

always_comb begin
    state_d = state_q;
    in_ready_o = 0;

    queued_valid_d = queued_valid_q;
    queued_operation_d = queued_operation_q;
    queued_vector_select_d = queued_vector_select_q;
    queued_go_matrix_address_d = queued_go_matrix_address_q;

    tmatmul_operation_d = tmatmul_operation_q;
    exportvector_counter_d = exportvector_counter_q;
    ddr_stream_address_d = ddr_stream_address_q;
    importvector_request_addr_counter_d = importvector_request_addr_counter_q;

    importvector_counter_d = importvector_counter_q;
    vector_select_d = vector_select_q;
    import_bram_addr_d = import_bram_addr_q;

    vector_request_valid_o = 0;
    vector_request_write_not_read_o = 'x;
    vector_request_vector_select_o = 'x;
    vector_request_vector_addr_o = 'x;
    vector_request_w_data_o = 'x;
    vector_read_ready_o = 0;

    ddr_stream_address_o = 'x;

    ddr_stream_valid_o = 0;
    ddr_r_ready_o = 0;

    accumulator_in_valid = 0;
    accumulator_in_final = 0;
    accumulator_operands = 'x;
    accumulator_out_ready = 0;

    importvector_request_valid = '0;
    importvector_request_write_not_read = 'x;
    importvector_request_addr = 'x;
    importvector_request_w_data = 'x;
    importvector_read_ready = '0;

    exportvector_request_valid = 0;
    exportvector_request_write_not_read = 'x;
    exportvector_request_addr = 'x;
    exportvector_request_w_data = 'x;
    exportvector_read_ready = 0;

    gbfifo_import_in_valid = 0;
    gbfifo_import_in_data = 'x;
    gbfifo_import_out_ready = 0;

    gbfifo_export_in_valid = 0;
    gbfifo_export_in_data = 'x;
    gbfifo_export_out_ready = 0;

    if (state_q == WAITING_FOR_IN) begin

        in_ready_o = !queued_valid_q;

        importvector_counter_d = 0;
        exportvector_counter_d = 0;
        importvector_request_addr_counter_d = 0;
        import_bram_addr_d = 0;

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

        ddr_stream_valid_o = 1;
        if (ddr_stream_ready_i) begin
            ddr_stream_address_o = ddr_stream_address_q;
            state_d = WORKING;
        end

    end else if ((state_q == WORKING) && (tmatmul_operation_q == ternip_pkg::IMPORT)) begin

        // Request chunks from vector registers
        if (importvector_counter_q < ternip_types#(Cfg)::NumChunksPerVector) begin
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

        // Request read from importvector
        if (importvector_counter_q < DdrReadsPerMatrix) begin
            importvector_request_valid = '1;
            importvector_request_write_not_read = '0;
            importvector_request_addr = importvector_request_addr_counter_q;
            if (importvector_request_ready) begin
                importvector_counter_d++;
                importvector_request_addr_counter_d++;
                if (importvector_request_addr_counter_q == DdrReadsPerRow-1) importvector_request_addr_counter_d = 0;
            end
        end

        // Receive read response from importvector, and fetch from ddr
        // add product to accumulator
        if (importvector_read_valid) begin
            ddr_r_ready_o = accumulator_in_ready[0];
            if (ddr_r_ready_o && ddr_r_valid_i) begin
                importvector_read_ready = '1;

                accumulator_in_valid = 1;
                for (int i = 0; i < Cfg.TmatmulParallelism; i++) begin
                    accumulator_operands[i / Cfg.D][i % Cfg.D] = ternary_mul(ddr_r_data_i[i], importvector_read_data[i % Cfg.D]);
                end
                if (importvector_read_addr >= DdrReadsPerRow-1) begin
                    accumulator_in_final = 1;
                end
            end
        end

        // Feed accumulator results into adapter
        accumulator_out_ready = gbfifo_export_in_ready;
        gbfifo_export_in_valid = accumulator_out_valid[0];
        gbfifo_export_in_data = accumulator_result;

        // Write serialized chunks to exportvector
        gbfifo_export_out_ready = exportvector_request_ready;
        if (gbfifo_export_out_ready && gbfifo_export_out_valid) begin
            exportvector_request_valid = 1;
            exportvector_request_write_not_read = 1;
            exportvector_request_addr = exportvector_counter_q;
            exportvector_request_w_data = gbfifo_export_out_data;

            exportvector_counter_d++;
            if (exportvector_counter_q >= ternip_types#(Cfg)::NumChunksPerVector-1) begin
                state_d = WAITING_FOR_IN;
                tmatmul_operation_d = ternip_pkg::NO_TMATMUL_OP;
                exportvector_counter_d = 0;
            end
        end

    end else if ((state_q == WORKING) && (tmatmul_operation_q == ternip_pkg::EXPORT)) begin

        // Read chunks from exportvector
        if (exportvector_counter_q < ternip_types#(Cfg)::NumChunksPerVector) begin
            exportvector_request_valid = 1;
            exportvector_request_write_not_read = 0;
            exportvector_request_addr = exportvector_counter_q;
            if (exportvector_request_ready) exportvector_counter_d++;
        end

        // Write chunks to vector registers
        exportvector_read_ready = vector_request_ready_i;
        if (exportvector_read_ready && exportvector_read_valid) begin
            vector_request_valid_o = 1;
            vector_request_write_not_read_o = '1;
            vector_request_vector_select_o = vector_select_q;
            vector_request_vector_addr_o = exportvector_read_addr;
            vector_request_w_data_o = exportvector_read_data;
            if (exportvector_read_addr >= ternip_types#(Cfg)::NumChunksPerVector-1) begin
                state_d = WAITING_FOR_IN;
                tmatmul_operation_d = ternip_pkg::NO_TMATMUL_OP;
            end
        end

    end
end

always_ff @(posedge clk_i) begin
    if (!rst_ni) begin
        queued_valid_q <= '0;
        state_q <= WAITING_FOR_IN;
    end else begin
        queued_valid_q <= queued_valid_d;
        state_q <= state_d;
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
    import_bram_addr_q <= import_bram_addr_d;
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
        import_bram_addr_q <= 'x;
    end
    `endif
end
always_ff @(posedge clk_i) begin
    ddr_stream_address_q <= ddr_stream_address_d;
    `ifndef SYNTHESIS
    if (!rst_ni) begin
        ddr_stream_address_q <= 'x;
    end
    `endif
end

endmodule

`undef SAFE_CLOG2
