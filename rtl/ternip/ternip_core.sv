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

// ternip_core
//
// Top-level Ternip compute core.
//
// The core accepts decoded instructions on instruction_ready_o/instruction_valid_i
// and dispatches each instruction to one functional unit: load/store, rowwise
// operation, ternary matmul, RMS, or stall. A shared vector-register file is
// arbitrated among the FUs, while load/store and tmatmul expose separate DDR
// stream/read/write interfaces.
//
// Use this module as the main RTL integration point. Drive one instruction when
// instruction_ready_o is high, connect the DDR stream ports to the memory system,
// and use stall_active_o/stall_clear_i to implement host-visible stalls or
// interrupts.

module ternip_core #(
    parameter int D                           = ternip_pkg::D,
    parameter int TmatmulParallelism          = ternip_pkg::TmatmulParallelism,
    parameter int FixedPointPrecision         = ternip_pkg::FixedPointPrecision,
    parameter int FixedPointExponent          = ternip_pkg::FixedPointExponent,
    parameter int VectorParallelism           = ternip_pkg::VectorParallelism,
    parameter int LutParallelism              = ternip_pkg::LutParallelism,
    parameter int NumVectorRegisters          = ternip_pkg::NumVectorRegisters,
    parameter int NumChunksPerVector          = ternip_pkg::NumChunksPerVector,
    parameter int ImmediateWidth              = ternip_pkg::ImmediateWidth,
    parameter int DdrAddressWidth             = ternip_pkg::DdrAddressWidth,
    parameter int BatchSize                   = ternip_pkg::BatchSize,
    parameter bit UseHardSigmoid              = ternip_pkg::UseHardSigmoid,
    parameter int RmsSqaSumPrecision          = ternip_pkg::RmsSqaSumPrecision,
    parameter int RmsSqaSumExponent           = ternip_pkg::RmsSqaSumExponent,
    parameter int RmsValueReciprocalPrecision = ternip_pkg::RmsValueReciprocalPrecision,
    parameter int RmsValueReciprocalExponent  = ternip_pkg::RmsValueReciprocalExponent,
    parameter int RmsSqrtInputPrecision       = ternip_pkg::RmsSqrtInputPrecision,
    parameter int RmsSqrtInputExponent        = ternip_pkg::RmsSqrtInputExponent,
    parameter int RmsAccumulatorWidth         = ternip_pkg::RmsAccumulatorWidth,
    parameter int MatrixSizeInBytes           = ternip_pkg::MatrixSizeInBytes,
    parameter int NumDdrBanksPerTmatmul       = ternip_pkg::NumDdrBanksPerTmatmul,

    parameter ternip_pkg::mul_impl_e MultiplicationImplementation = ternip_pkg::MultiplicationImplementation,
    parameter ternip_pkg::div_impl_e DivisionImplementation       = ternip_pkg::DivisionImplementation,

    parameter type instruction_t       = ternip_pkg::instruction_t,

    localparam type fixed_point_t         = logic signed [ternip_pkg::FixedPointPrecision-1:0],
    localparam type vector_chunk_t        = fixed_point_t [VectorParallelism-1:0],
    localparam type ternary_t             = logic signed [1:0],
    localparam type tmatmul_stream_data_t = ternary_t [TmatmulParallelism-1:0],
    localparam type vector_offset_t       = logic [$clog2(NumChunksPerVector)-1:0],
    localparam type vector_select_t       = logic [$clog2(NumVectorRegisters)-1:0],
    localparam type immediate_t           = logic [ImmediateWidth-1:0],
    localparam type ddr_address_t         = logic [DdrAddressWidth-1:0]
) (
    input  logic                 clk_i,
    input  logic                 rst_ni,

    output logic                 instruction_ready_o,
    input  logic                 instruction_valid_i,
    input  instruction_t         instruction_i,

    input  logic                 loadstore_ddr_stream_ready_i,
    output logic                 loadstore_ddr_stream_valid_o,
    output ddr_address_t         loadstore_ddr_stream_address_o,
    output logic                 loadstore_ddr_stream_write_not_read_o,
    output logic [31:0]          loadstore_ddr_stream_length_o,

    output logic                 loadstore_ddr_r_ready_o,
    input  logic                 loadstore_ddr_r_valid_i,
    input  vector_chunk_t        loadstore_ddr_r_data_i,

    input  logic                 loadstore_ddr_w_ready_i,
    output logic                 loadstore_ddr_w_valid_o,
    output vector_chunk_t        loadstore_ddr_w_data_o,

    output logic [63:0]          loadstore_ddr_debug_o,

    input  logic         [NumDdrBanksPerTmatmul-1:0]         tmatmul_ddr_stream_ready_i,
    output logic         [NumDdrBanksPerTmatmul-1:0]         tmatmul_ddr_stream_valid_o,
    output ddr_address_t [NumDdrBanksPerTmatmul-1:0]         tmatmul_ddr_stream_address_o,
    output logic         [NumDdrBanksPerTmatmul-1:0][31:0]   tmatmul_ddr_stream_length_o,

    output logic                 [NumDdrBanksPerTmatmul-1:0] tmatmul_ddr_r_ready_o,
    input  logic                 [NumDdrBanksPerTmatmul-1:0] tmatmul_ddr_r_valid_i,
    input  tmatmul_stream_data_t [NumDdrBanksPerTmatmul-1:0] tmatmul_ddr_r_data_i,

    output logic                 stall_active_o,
    input  logic                 stall_clear_i
);

logic           vector_request_ready;
logic           vector_request_valid;
logic           vector_request_write_not_read;
vector_select_t vector_request_vector_select;
vector_offset_t vector_request_vector_addr;
vector_chunk_t  vector_request_w_data;

logic           vector_read_ready;
logic           vector_read_valid;
vector_offset_t vector_read_addr;
vector_chunk_t  vector_read_data;

ternip_vector_registers #(
    .D(D),
    .FixedPointPrecision(FixedPointPrecision),
    .VectorParallelism(VectorParallelism),
    .NumVectorRegisters(NumVectorRegisters),
    .NumChunksPerVector(NumChunksPerVector)
) vector_registers (
    .clk_i,
    .rst_ni,

    .request_ready_o(vector_request_ready),
    .request_valid_i(vector_request_valid),
    .request_write_not_read_i(vector_request_write_not_read),
    .request_vector_select_i(vector_request_vector_select),
    .request_vector_addr_i(vector_request_vector_addr),
    .request_w_data_i(vector_request_w_data),

    .read_ready_i(vector_read_ready),
    .read_valid_o(vector_read_valid),
    .read_vector_select_o(),
    .read_addr_o(vector_read_addr),
    .read_data_o(vector_read_data)
);

logic                      loadstore_in_ready;
logic                      loadstore_in_valid;
ternip_pkg::loadstore_op_e loadstore_in_vector_operation;
ddr_address_t              loadstore_in_vector_memory_address;
vector_select_t            loadstore_in_vector_select;

logic                      loadstore_vector_request_valid;
logic                      loadstore_vector_request_write_not_read;
vector_select_t            loadstore_vector_request_vector_select;
vector_offset_t            loadstore_vector_request_vector_addr;
vector_chunk_t             loadstore_vector_request_w_data;

logic                      loadstore_vector_read_ready;

ternip_loadstore #(
    .D(D),
    .FixedPointPrecision(FixedPointPrecision),
    .VectorParallelism(VectorParallelism),
    .NumVectorRegisters(NumVectorRegisters),
    .NumChunksPerVector(NumChunksPerVector),
    .DdrAddressWidth(DdrAddressWidth),
    .BatchSize(BatchSize)
) loadstore (
    .clk_i,
    .rst_ni,

    .in_ready_o(loadstore_in_ready),
    .in_valid_i(loadstore_in_valid),
    .in_vector_operation_i(loadstore_in_vector_operation),
    .in_vector_memory_address_i(loadstore_in_vector_memory_address),
    .in_vector_select_i(loadstore_in_vector_select),

    .vector_request_ready_i(vector_request_ready),
    .vector_request_valid_o(loadstore_vector_request_valid),
    .vector_request_write_not_read_o(loadstore_vector_request_write_not_read),
    .vector_request_vector_select_o(loadstore_vector_request_vector_select),
    .vector_request_vector_addr_o(loadstore_vector_request_vector_addr),
    .vector_request_w_data_o(loadstore_vector_request_w_data),

    .vector_read_ready_o(loadstore_vector_read_ready),
    .vector_read_valid_i(vector_read_valid),
    .vector_read_addr_i(vector_read_addr),
    .vector_read_data_i(vector_read_data),

    .ddr_stream_ready_i(loadstore_ddr_stream_ready_i),
    .ddr_stream_valid_o(loadstore_ddr_stream_valid_o),
    .ddr_stream_address_o(loadstore_ddr_stream_address_o),
    .ddr_stream_write_not_read_o(loadstore_ddr_stream_write_not_read_o),
    .ddr_stream_length_o(loadstore_ddr_stream_length_o),

    .ddr_r_ready_o(loadstore_ddr_r_ready_o),
    .ddr_r_valid_i(loadstore_ddr_r_valid_i),
    .ddr_r_data_i(loadstore_ddr_r_data_i),

    .ddr_w_ready_i(loadstore_ddr_w_ready_i),
    .ddr_w_valid_o(loadstore_ddr_w_valid_o),
    .ddr_w_data_o(loadstore_ddr_w_data_o),

    .debug_o(loadstore_ddr_debug_o)
);

logic                    rowwise_operation_in_ready;
logic                    rowwise_operation_in_valid;
ternip_pkg::rowwise_op_e rowwise_operation_in_operation;

vector_select_t          rowwise_operation_in_vector1_r_select;
vector_select_t          rowwise_operation_in_vector2_r_select;
vector_select_t          rowwise_operation_in_vector3_r_select;

logic                    rowwise_operation_vector_request_valid;
logic                    rowwise_operation_vector_request_write_not_read;
vector_select_t          rowwise_operation_vector_request_vector_select;
vector_offset_t          rowwise_operation_vector_request_vector_addr;
vector_chunk_t           rowwise_operation_vector_request_w_data;

logic                    rowwise_operation_vector_read_ready;

ternip_rowwise_operation #(
    .D(D),
    .FixedPointPrecision(FixedPointPrecision),
    .FixedPointExponent(FixedPointExponent),
    .VectorParallelism(VectorParallelism),
    .LutParallelism(LutParallelism),
    .NumVectorRegisters(NumVectorRegisters),
    .NumChunksPerVector(NumChunksPerVector),
    .UseHardSigmoid(UseHardSigmoid),
    .MultiplicationImplementation(MultiplicationImplementation)
) rowwise_operation (
    .clk_i,
    .rst_ni,

    .in_ready_o(rowwise_operation_in_ready),
    .in_valid_i(rowwise_operation_in_valid),
    .in_operation_i(rowwise_operation_in_operation),

    .in_vector1_r_select_i(rowwise_operation_in_vector1_r_select),
    .in_vector2_r_select_i(rowwise_operation_in_vector2_r_select),
    .in_vector3_r_select_i(rowwise_operation_in_vector3_r_select),

    .vector_request_ready_i(vector_request_ready),
    .vector_request_valid_o(rowwise_operation_vector_request_valid),
    .vector_request_write_not_read_o(rowwise_operation_vector_request_write_not_read),
    .vector_request_vector_select_o(rowwise_operation_vector_request_vector_select),
    .vector_request_vector_addr_o(rowwise_operation_vector_request_vector_addr),
    .vector_request_w_data_o(rowwise_operation_vector_request_w_data),

    .vector_read_ready_o(rowwise_operation_vector_read_ready),
    .vector_read_valid_i(vector_read_valid),
    .vector_read_data_i(vector_read_data)
);

logic                rms_in_ready;
logic                rms_in_valid;
ternip_pkg::rms_op_e rms_in_op;
vector_select_t      rms_in_vector1_select;
vector_select_t      rms_in_vector2_select;
immediate_t          rms_in_length;

logic                rms_vector_request_valid;
logic                rms_vector_request_write_not_read;
vector_select_t      rms_vector_request_vector_select;
vector_offset_t      rms_vector_request_vector_addr;
vector_chunk_t       rms_vector_request_w_data;

logic                rms_vector_read_ready;

ternip_rms #(
    .FixedPointPrecision(FixedPointPrecision),
    .FixedPointExponent(FixedPointExponent),
    .VectorParallelism(VectorParallelism),
    .NumVectorRegisters(NumVectorRegisters),
    .NumChunksPerVector(NumChunksPerVector),
    .ImmediateWidth(ImmediateWidth),
    .RmsSqaSumPrecision(RmsSqaSumPrecision),
    .RmsSqaSumExponent(RmsSqaSumExponent),
    .RmsValueReciprocalPrecision(RmsValueReciprocalPrecision),
    .RmsValueReciprocalExponent(RmsValueReciprocalExponent),
    .RmsSqrtInputPrecision(RmsSqrtInputPrecision),
    .RmsSqrtInputExponent(RmsSqrtInputExponent),
    .RmsAccumulatorWidth(RmsAccumulatorWidth),
    .DivisionImplementation(DivisionImplementation)
) rms (
    .clk_i,
    .rst_ni,

    .in_ready_o(rms_in_ready),
    .in_valid_i(rms_in_valid),
    .in_rms_op_i(rms_in_op),
    .in_vector1_select_i(rms_in_vector1_select),
    .in_vector2_select_i(rms_in_vector2_select),
    .in_rms_length_i(rms_in_length),

    .vector_request_ready_i(vector_request_ready),
    .vector_request_valid_o(rms_vector_request_valid),
    .vector_request_write_not_read_o(rms_vector_request_write_not_read),
    .vector_request_vector_select_o(rms_vector_request_vector_select),
    .vector_request_vector_addr_o(rms_vector_request_vector_addr),
    .vector_request_w_data_o(rms_vector_request_w_data),

    .vector_read_ready_o(rms_vector_read_ready),
    .vector_read_valid_i(vector_read_valid),
    .vector_read_addr_i(vector_read_addr),
    .vector_read_data_i(vector_read_data),

    .accumulator_out_valid_o(),
    .accumulator_out_result_o(),
    .rms_value_reciprocal_o(),
    .rms_value_reciprocal_valid_o()
);

logic                                     tmatmul_in_ready;
logic                                     tmatmul_in_valid;
vector_select_t                           tmatmul_in_vector_select;
ddr_address_t [NumDdrBanksPerTmatmul-1:0] tmatmul_in_go_matrix_address;

ternip_pkg::tmatmul_op_e tmatmul_in_operation;

logic                    tmatmul_vector_request_valid;
logic                    tmatmul_vector_request_write_not_read;
vector_select_t          tmatmul_vector_request_vector_select;
vector_offset_t          tmatmul_vector_request_vector_addr;
vector_chunk_t           tmatmul_vector_request_w_data;

logic                    tmatmul_vector_read_ready;

ternip_tmatmul #(
    .D(D),
    .TmatmulParallelism(TmatmulParallelism),
    .FixedPointPrecision(FixedPointPrecision),
    .VectorParallelism(VectorParallelism),
    .NumVectorRegisters(NumVectorRegisters),
    .NumChunksPerVector(NumChunksPerVector),
    .DdrAddressWidth(DdrAddressWidth),
    .MatrixSizeInBytes(MatrixSizeInBytes),
    .NumDdrBanksPerTmatmul(NumDdrBanksPerTmatmul)
) tmatmul (
    .clk_i,
    .rst_ni,

    .in_ready_o(tmatmul_in_ready),
    .in_valid_i(tmatmul_in_valid),
    .in_operation_i(tmatmul_in_operation),
    .in_go_matrix_address_i(tmatmul_in_go_matrix_address),
    .in_vector_select_i(tmatmul_in_vector_select),

    .vector_request_ready_i(vector_request_ready),
    .vector_request_valid_o(tmatmul_vector_request_valid),
    .vector_request_write_not_read_o(tmatmul_vector_request_write_not_read),
    .vector_request_vector_select_o(tmatmul_vector_request_vector_select),
    .vector_request_vector_addr_o(tmatmul_vector_request_vector_addr),
    .vector_request_w_data_o(tmatmul_vector_request_w_data),

    .vector_read_ready_o(tmatmul_vector_read_ready),
    .vector_read_valid_i(vector_read_valid),
    .vector_read_addr_i(vector_read_addr),
    .vector_read_data_i(vector_read_data),

    .ddr_stream_ready_i(tmatmul_ddr_stream_ready_i),
    .ddr_stream_valid_o(tmatmul_ddr_stream_valid_o),
    .ddr_stream_address_o(tmatmul_ddr_stream_address_o),
    .ddr_stream_length_o(tmatmul_ddr_stream_length_o),

    .ddr_r_ready_o(tmatmul_ddr_r_ready_o),
    .ddr_r_valid_i(tmatmul_ddr_r_valid_i),
    .ddr_r_data_i(tmatmul_ddr_r_data_i)
);

logic received_stall_command;
logic stall_active_q;

always_ff @(posedge clk_i) begin
    if (!rst_ni) begin
        stall_active_q <= 0;
    end else begin
        if (received_stall_command) begin
            stall_active_q <= 1;
        end else if (stall_clear_i) begin
            stall_active_q <= 0;
        end
    end
end

assign stall_active_o = stall_active_q;

// Variable-length instruction decode FSM.
//
// Each instruction starts with a 64-bit opcode beat on `instruction_i`.
// Address-carrying instructions stream their addresses as trailing 64-bit
// beats on the same channel:
//   * ldv / sv (LOADSTORE): 1 trailing address beat (vector_memory_address).
//   * tmatmul_go: NumDdrBanksPerTmatmul trailing beats, one DDR address per
//     bank.
//   * everything else: opcode beat only.
// All other immediates (rms_finish_accumulate's length) ride inside the
// opcode beat's `immediate` field.
//
// The FSM decodes the opcode in DECODE, collects the trailing beats in
// FETCH_LOADSTORE_ADDR / FETCH_TMATMUL_ADDR, and then asserts the target
// FU's `in_valid` in DISPATCH until the FU consumes it.

`define SAFE_CLOG2(x) ( (((x)==1) || ((x)==0))? 1 : $clog2((x)))

typedef enum logic [1:0] {
    INSTR_FSM_DECODE,
    INSTR_FSM_FETCH_LOADSTORE_ADDR,
    INSTR_FSM_FETCH_TMATMUL_ADDR,
    INSTR_FSM_DISPATCH
} instr_fsm_e;

instr_fsm_e   instr_fsm_d, instr_fsm_q;
instruction_t latched_instr_d, latched_instr_q;
ddr_address_t                              latched_loadstore_addr_d, latched_loadstore_addr_q;
ddr_address_t [NumDdrBanksPerTmatmul-1:0]  latched_tmatmul_addrs_d,  latched_tmatmul_addrs_q;
logic [`SAFE_CLOG2(NumDdrBanksPerTmatmul):0] tmatmul_addr_counter_d, tmatmul_addr_counter_q;

// A 64-bit beat on `instruction_i`, reinterpreted as a raw DDR address. The
// FSM consumes it during the FETCH_* states.
wire ddr_address_t instr_beat_as_addr = ddr_address_t'(instruction_i);

// Gate opcode acceptance on all FUs being ready, exactly like the original
// single-beat dispatch path did. The vector_request mux assumes only one FU
// is doing a register access at a time; if we let a new opcode in before
// the previous instruction's FU finished, two FUs end up racing the same
// cycle for the vector_register port.
wire all_fus_in_ready = loadstore_in_ready
                      & rms_in_ready
                      & rowwise_operation_in_ready
                      & tmatmul_in_ready;

logic instr_ready_internal;
assign instruction_ready_o = instr_ready_internal & !stall_active_q;

always_comb begin
    vector_request_valid = 0;
    vector_request_write_not_read = 'x;
    vector_request_vector_select = 'x;
    vector_request_vector_addr = 'x;
    vector_request_w_data = 'x;

    unique case (1)
        loadstore_vector_request_valid: begin
            vector_request_valid = 1;
            vector_request_write_not_read = loadstore_vector_request_write_not_read;
            vector_request_vector_select = loadstore_vector_request_vector_select;
            vector_request_vector_addr = loadstore_vector_request_vector_addr;
            vector_request_w_data = loadstore_vector_request_w_data;
        end
        rms_vector_request_valid: begin
            vector_request_valid = 1;
            vector_request_write_not_read = rms_vector_request_write_not_read;
            vector_request_vector_select = rms_vector_request_vector_select;
            vector_request_vector_addr = rms_vector_request_vector_addr;
            vector_request_w_data = rms_vector_request_w_data;
        end
        rowwise_operation_vector_request_valid: begin
            vector_request_valid = 1;
            vector_request_write_not_read = rowwise_operation_vector_request_write_not_read;
            vector_request_vector_select = rowwise_operation_vector_request_vector_select;
            vector_request_vector_addr = rowwise_operation_vector_request_vector_addr;
            vector_request_w_data = rowwise_operation_vector_request_w_data;
        end
        tmatmul_vector_request_valid: begin
            vector_request_valid = 1;
            vector_request_write_not_read = tmatmul_vector_request_write_not_read;
            vector_request_vector_select = tmatmul_vector_request_vector_select;
            vector_request_vector_addr = tmatmul_vector_request_vector_addr;
            vector_request_w_data = tmatmul_vector_request_w_data;
        end
        default: ;
    endcase
end

assign vector_read_ready = (loadstore_vector_read_ready
                            | rms_vector_read_ready
                            | rowwise_operation_vector_read_ready
                            | tmatmul_vector_read_ready);

`ifndef SYNTHESIS
always @(posedge clk_i) if (rst_ni) begin
    assert final (1 >= $countones({
        loadstore_vector_request_valid,
        rms_vector_request_valid,
        rowwise_operation_vector_request_valid,
        tmatmul_vector_request_valid
    })) else $fatal(0, "Conflict in vector_request_valid");
    assert final (1 >= $countones({
        loadstore_vector_read_ready,
        rms_vector_read_ready,
        rowwise_operation_vector_read_ready,
        tmatmul_vector_read_ready
    })) else $fatal(0, "Conflict in vector_read_ready");
end
`endif

always_comb begin
    instr_fsm_d = instr_fsm_q;
    latched_instr_d = latched_instr_q;
    latched_loadstore_addr_d = latched_loadstore_addr_q;
    latched_tmatmul_addrs_d = latched_tmatmul_addrs_q;
    tmatmul_addr_counter_d = tmatmul_addr_counter_q;

    instr_ready_internal = 0;

    loadstore_in_valid = '0;
    loadstore_in_vector_operation = ternip_pkg::NO_LS_OP;
    loadstore_in_vector_memory_address = 'x;
    loadstore_in_vector_select = 'x;

    rowwise_operation_in_valid = 0;
    rowwise_operation_in_operation = ternip_pkg::NOP;
    rowwise_operation_in_vector1_r_select = 'x;
    rowwise_operation_in_vector2_r_select = 'x;
    rowwise_operation_in_vector3_r_select = 'x;

    rms_in_valid = 0;
    rms_in_op = ternip_pkg::NO_RMS_OP;
    rms_in_vector1_select = 'x;
    rms_in_vector2_select = 'x;
    rms_in_length = 'x;

    tmatmul_in_valid = 0;
    tmatmul_in_operation = ternip_pkg::NO_TMATMUL_OP;
    tmatmul_in_go_matrix_address = '{default: 'x};
    tmatmul_in_vector_select = 'x;

    received_stall_command = 0;

    unique case (instr_fsm_q)
        INSTR_FSM_DECODE: begin
            // Ready for a fresh opcode beat — but only when every FU is
            // idle, matching the original combinational dispatch path.
            instr_ready_internal = all_fus_in_ready;
            if (instruction_valid_i && instruction_ready_o) begin
                latched_instr_d = instruction_i;
                unique case (instruction_i.fu)
                    ternip_pkg::LOADSTORE: instr_fsm_d = INSTR_FSM_FETCH_LOADSTORE_ADDR;
                    ternip_pkg::TMATMUL: begin
                        if (instruction_i.tmatmul_op == ternip_pkg::GO) begin
                            instr_fsm_d = INSTR_FSM_FETCH_TMATMUL_ADDR;
                            tmatmul_addr_counter_d = 0;
                        end else begin
                            instr_fsm_d = INSTR_FSM_DISPATCH;
                        end
                    end
                    default: instr_fsm_d = INSTR_FSM_DISPATCH;
                endcase
            end
        end

        INSTR_FSM_FETCH_LOADSTORE_ADDR: begin
            // Consume the single trailing address beat for ldv / sv.
            instr_ready_internal = 1;
            if (instruction_valid_i && instruction_ready_o) begin
                latched_loadstore_addr_d = instr_beat_as_addr;
                instr_fsm_d = INSTR_FSM_DISPATCH;
            end
        end

        INSTR_FSM_FETCH_TMATMUL_ADDR: begin
            // Consume N trailing address beats for tmatmul_go, one per bank.
            instr_ready_internal = 1;
            if (instruction_valid_i && instruction_ready_o) begin
                latched_tmatmul_addrs_d[tmatmul_addr_counter_q[`SAFE_CLOG2(NumDdrBanksPerTmatmul)-1:0]] = instr_beat_as_addr;
                if (tmatmul_addr_counter_q == NumDdrBanksPerTmatmul - 1) begin
                    instr_fsm_d = INSTR_FSM_DISPATCH;
                end else begin
                    tmatmul_addr_counter_d = tmatmul_addr_counter_q + 1;
                end
            end
        end

        INSTR_FSM_DISPATCH: begin
            // Drive the latched instruction at the chosen FU until it accepts.
            unique case (latched_instr_q.fu)
                ternip_pkg::LOADSTORE: begin
                    loadstore_in_valid = 1;
                    loadstore_in_vector_operation = latched_instr_q.loadstore_op;
                    loadstore_in_vector_memory_address = latched_loadstore_addr_q;
                    loadstore_in_vector_select = latched_instr_q.v_a;
                    if (loadstore_in_ready) instr_fsm_d = INSTR_FSM_DECODE;
                end
                ternip_pkg::ROWWISE_OPERATION: begin
                    rowwise_operation_in_valid = 1;
                    rowwise_operation_in_operation = latched_instr_q.rowwise_op;
                    rowwise_operation_in_vector1_r_select = latched_instr_q.v_a;
                    rowwise_operation_in_vector2_r_select = latched_instr_q.v_b;
                    rowwise_operation_in_vector3_r_select = latched_instr_q.v_y;
                    if (rowwise_operation_in_ready) instr_fsm_d = INSTR_FSM_DECODE;
                end
                ternip_pkg::TMATMUL: begin
                    tmatmul_in_valid = 1;
                    tmatmul_in_operation = latched_instr_q.tmatmul_op;
                    tmatmul_in_go_matrix_address = latched_tmatmul_addrs_q;
                    tmatmul_in_vector_select = latched_instr_q.v_a;
                    if (tmatmul_in_ready) instr_fsm_d = INSTR_FSM_DECODE;
                end
                ternip_pkg::RMS: begin
                    rms_in_valid = 1;
                    rms_in_op = latched_instr_q.rms_op;
                    rms_in_vector1_select = latched_instr_q.v_a;
                    rms_in_vector2_select = latched_instr_q.v_y;
                    rms_in_length = latched_instr_q.immediate;
                    if (rms_in_ready) instr_fsm_d = INSTR_FSM_DECODE;
                end
                ternip_pkg::STALL: begin
                    received_stall_command = 1;
                    instr_fsm_d = INSTR_FSM_DECODE;
                end
                default: instr_fsm_d = INSTR_FSM_DECODE;
            endcase
        end

        default: instr_fsm_d = INSTR_FSM_DECODE;
    endcase
end

always_ff @(posedge clk_i) begin
    if (!rst_ni) begin
        instr_fsm_q <= INSTR_FSM_DECODE;
        tmatmul_addr_counter_q <= '0;
    end else begin
        instr_fsm_q <= instr_fsm_d;
        tmatmul_addr_counter_q <= tmatmul_addr_counter_d;
    end
end
always_ff @(posedge clk_i) begin
    latched_instr_q <= latched_instr_d;
    latched_loadstore_addr_q <= latched_loadstore_addr_d;
    latched_tmatmul_addrs_q <= latched_tmatmul_addrs_d;
    `ifndef SYNTHESIS
    if (!rst_ni) begin
        latched_instr_q <= 'x;
        latched_loadstore_addr_q <= 'x;
        latched_tmatmul_addrs_q <= '{default: 'x};
    end
    `endif
end

endmodule

`undef SAFE_CLOG2
