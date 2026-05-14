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

// ternip_rms
//
// Functional unit for RMS normalization over vector registers.
//
// CLEAR resets the internal accumulated square-sum register.
// ACCUMULATE adds the squared elements of one vector into that register.
// FINISH_ACCUMULATE computes the reciprocal RMS scale factor.
// NORM multiplies a vector by that scale and writes the normalized result.
//
// Use CLEAR before a new RMS group. Then issue one or more ACCUMULATE commands,
// one FINISH_ACCUMULATE command, and one or more NORM commands. The unit accepts
// one command on in_* and sequences the vector-register and math handshakes
// until that command finishes. NORM uses the primary vector port for reads and
// vector_request2_* for writes so reads and writes can overlap when the register
// file is dual-ported.

module ternip_rms #(
    parameter int FixedPointPrecision         = ternip_pkg::FixedPointPrecision,
    parameter int FixedPointExponent          = ternip_pkg::FixedPointExponent,
    parameter int VectorParallelism           = ternip_pkg::VectorParallelism,
    parameter int NumVectorRegisters          = ternip_pkg::NumVectorRegisters,
    parameter int NumChunksPerVector          = ternip_pkg::NumChunksPerVector,
    parameter int ImmediateWidth              = ternip_pkg::ImmediateWidth,
    parameter int RmsSqaSumPrecision          = ternip_pkg::RmsSqaSumPrecision,
    parameter int RmsSqaSumExponent           = ternip_pkg::RmsSqaSumExponent,
    parameter int RmsValueReciprocalPrecision = ternip_pkg::RmsValueReciprocalPrecision,
    parameter int RmsValueReciprocalExponent  = ternip_pkg::RmsValueReciprocalExponent,
    parameter int RmsSqrtInputPrecision       = ternip_pkg::RmsSqrtInputPrecision,
    parameter int RmsSqrtInputExponent        = ternip_pkg::RmsSqrtInputExponent,
    parameter int RmsAccumulatorWidth         = ternip_pkg::RmsAccumulatorWidth,

    parameter ternip_pkg::div_impl_e DivisionImplementation = ternip_pkg::DivisionImplementation,

    localparam type fixed_point_t          = logic signed [ternip_pkg::FixedPointPrecision-1:0],
    localparam type vector_chunk_t         = fixed_point_t [VectorParallelism-1:0],
    localparam type vector_offset_t        = logic [$clog2(NumChunksPerVector)-1:0],
    localparam type vector_select_t        = logic [$clog2(NumVectorRegisters)-1:0],
    localparam type immediate_t            = logic [ImmediateWidth-1:0],
    localparam type rms_sqa_sum_t          = logic signed [RmsSqaSumPrecision-1:0],
    localparam type rms_accumulator_t      = logic signed [RmsAccumulatorWidth-1:0],
    localparam type rms_value_reciprocal_t = logic signed [RmsValueReciprocalPrecision-1:0],
    localparam type rms_sqrt_input_t       = logic signed [RmsSqrtInputPrecision-1:0]
) (
    input  logic                clk_i,
    input  logic                rst_ni,

    output logic                in_ready_o,
    input  logic                in_valid_i,
    input  ternip_pkg::rms_op_e in_rms_op_i,
    input  vector_select_t      in_vector1_select_i,
    input  vector_select_t      in_vector2_select_i,
    input  immediate_t          in_rms_length_i,

    input  logic           vector_request_ready_i,
    output logic           vector_request_valid_o,
    output logic           vector_request_write_not_read_o,
    output vector_select_t vector_request_vector_select_o,
    output vector_offset_t vector_request_vector_addr_o,
    output vector_chunk_t  vector_request_w_data_o,

    output logic           vector_read_ready_o,
    input  logic           vector_read_valid_i,
    input  vector_offset_t vector_read_addr_i,
    input  vector_chunk_t  vector_read_data_i,

    input  logic           vector_request2_ready_i,
    output logic           vector_request2_valid_o,
    output logic           vector_request2_write_not_read_o,
    output vector_select_t vector_request2_vector_select_o,
    output vector_offset_t vector_request2_vector_addr_o,
    output vector_chunk_t  vector_request2_w_data_o,

    // debug ports
    output logic                  accumulator_out_valid_o,
    output rms_accumulator_t      accumulator_out_result_o,
    output rms_value_reciprocal_t rms_value_reciprocal_o,
    output logic                  rms_value_reciprocal_valid_o
);

ternip_pkg::rms_op_e rms_op_d, rms_op_q;
vector_select_t      in_vector1_select_d, in_vector1_select_q;
vector_select_t      in_vector2_select_d, in_vector2_select_q;
immediate_t          in_rms_length_d, in_rms_length_q;

enum logic [1:0] {
    WAITING_FOR_IN,
    WORKING,
    SENDING_FINAL_TO_ACCUMULATOR,
    WAITING_FOR_ACCUMULATOR
} state_d, state_q = WAITING_FOR_IN;

logic [$clog2(NumChunksPerVector):0] vector_read_counter_d, vector_read_counter_q;
logic [$clog2(NumChunksPerVector):0] vector_processed_counter_d, vector_processed_counter_q;


// Accumulate

logic         [VectorParallelism-1:0] square_in_ready;
logic         [VectorParallelism-1:0] square_in_valid;
vector_chunk_t                        square_operand_in;

logic         [VectorParallelism-1:0] square_out_ready;
logic         [VectorParallelism-1:0] square_out_valid;
rms_sqa_sum_t [VectorParallelism-1:0] square_result_out;

`ifndef SYNTHESIS
// Verify that all square modules are synchronized
// If all square modules are synchronized, then only square[0]'s control signals need to be read
always_comb begin
    assert (!rst_ni || ((&square_in_ready) == (|square_in_ready)))
        else $fatal(0, "square_in_ready bits are not all uniform");
    assert (!rst_ni || ((&square_in_valid) == (|square_in_valid)))
        else $fatal(0, "square_in_valid bits are not all uniform");
    assert (!rst_ni || ((&square_out_ready) == (|square_out_ready)))
        else $fatal(0, "square_out_ready bits are not all uniform");
    assert (!rst_ni || ((&square_out_valid) == (|square_out_valid)))
        else $fatal(0, "square_out_valid bits are not all uniform");
end
`endif

for (genvar i_GEN = 0; i_GEN < VectorParallelism; i_GEN++) begin : parallel_squares

    ternip_mul #(
        .InAPrecision(FixedPointPrecision),
        .InAExponent(FixedPointExponent),
        .InBPrecision(FixedPointPrecision),
        .InBExponent(FixedPointExponent),
        .OutPrecision(RmsSqaSumPrecision),
        .OutExponent(RmsSqaSumExponent)
    ) square (
        .clk_i,
        .rst_ni,

        .in_ready_o(square_in_ready[i_GEN]),
        .in_valid_i(square_in_valid[i_GEN]),
        .a_i(square_operand_in[i_GEN]),
        .b_i(square_operand_in[i_GEN]),

        .out_ready_i(square_out_ready[i_GEN]),
        .out_valid_o(square_out_valid[i_GEN]),
        .y_o(square_result_out[i_GEN])
    );

end

logic accumulator_in_ready;
logic accumulator_in_valid;
logic accumulator_in_final;
rms_sqa_sum_t [VectorParallelism-1:0] accumulator_in_operand;

logic accumulator_out_ready;
logic accumulator_out_valid;
rms_accumulator_t accumulator_out_result;

always_ff @(posedge clk_i) begin
    if (!rst_ni) begin
        accumulator_out_valid_o <= 0;
    end else if (accumulator_out_valid) begin
        accumulator_out_valid_o <= 1;
    end
end
always_ff @(posedge clk_i) begin
    accumulator_out_result_o <= accumulator_out_result;
    `ifndef SYNTHESIS
    if (!rst_ni) begin
        accumulator_out_result_o <= 'x;
    end
    `endif
end

ternip_multioperand_accumulator #(
    .operand_t(rms_sqa_sum_t),
    .result_t(rms_accumulator_t),
    .NUM_OPERANDS(VectorParallelism),
    .NEXT_STAGE_FANIN(2)
) multioperand_accumulator (
    .clk_i,
    .rst_ni,

    .in_ready_o(accumulator_in_ready),
    .in_valid_i(accumulator_in_valid),
    .in_final_i(accumulator_in_final),
    .in_operands_i(accumulator_in_operand),

    .out_ready_i(accumulator_out_ready),
    .out_valid_o(accumulator_out_valid),
    .out_result_o(accumulator_out_result)
);


// Finish accumulate

// rms_value_reciprocal = 1 / SQRT( accumulator / rms_length )
// rms_value_reciprocal = SQRT( rms_length / accumulator )

rms_value_reciprocal_t rms_value_reciprocal_d, rms_value_reciprocal_q;
logic                  rms_value_reciprocal_valid_d, rms_value_reciprocal_valid_q;

assign rms_value_reciprocal_o       = rms_value_reciprocal_q;
assign rms_value_reciprocal_valid_o = rms_value_reciprocal_valid_q;

logic div_in_ready;
logic div_in_valid;
immediate_t div_in_dividend;
rms_accumulator_t div_in_divisor;

logic div_out_ready;
logic div_out_valid;
rms_sqrt_input_t div_out_quotient;

ternip_div #(
    .InAPrecision(ImmediateWidth),
    .InAExponent(0),
    .InBPrecision(RmsAccumulatorWidth),
    .InBExponent(RmsSqaSumExponent),
    .OutPrecision(RmsSqrtInputPrecision),
    .OutExponent(RmsSqrtInputExponent),
    .Implementation(DivisionImplementation)
) rms_value_reciprocal_divider (
    .clk_i,
    .rst_ni,

    .in_ready_o(div_in_ready),
    .in_valid_i(div_in_valid),
    .a_i(div_in_dividend),
    .b_i(div_in_divisor),

    .out_ready_i(div_out_ready),
    .out_valid_o(div_out_valid),
    .y_o(div_out_quotient)
);

logic                 rms_sqrt_in_ready;
wire logic            rms_sqrt_in_valid = div_out_valid;
wire rms_sqrt_input_t rms_sqrt_a = div_out_quotient;

logic                  rms_sqrt_out_ready;
logic                  rms_sqrt_out_valid;
rms_value_reciprocal_t rms_sqrt_y;

assign div_out_ready = rms_sqrt_in_ready;

ternip_sqrt #(
    .InPrecision(RmsSqrtInputPrecision),
    .InExponent(RmsSqrtInputExponent),
    .OutPrecision(RmsValueReciprocalPrecision),
    .OutExponent(RmsValueReciprocalExponent)
) sqrt (
    .clk_i,
    .rst_ni,
    .in_ready_o(rms_sqrt_in_ready),
    .in_valid_i(rms_sqrt_in_valid),
    .a_i(rms_sqrt_a),
    .out_ready_i(rms_sqrt_out_ready),
    .out_valid_o(rms_sqrt_out_valid),
    .y_o(rms_sqrt_y)
);

// Precision Playground
// https://www.desmos.com/calculator/z2je9ajel7


// Norm

logic [VectorParallelism-1:0] norm_mul_in_ready;
logic [VectorParallelism-1:0] norm_mul_in_valid;
vector_chunk_t                norm_mul_in_a;
rms_value_reciprocal_t        norm_mul_in_b;

logic [VectorParallelism-1:0] norm_mul_out_ready;
logic [VectorParallelism-1:0] norm_mul_out_valid;
vector_chunk_t                norm_mul_out_result;

vector_chunk_t norm_mul_out_result_buffer_d, norm_mul_out_result_buffer_q;
logic norm_mul_out_result_buffer_valid_d, norm_mul_out_result_buffer_valid_q;

`ifndef SYNTHESIS
// Verify that all norm_mul modules are synchronized
// If all norm_mul modules are synchronized, then only norm_mul[0]'s control signals need to be read
always_comb begin
    assert ((!rst_ni || (&norm_mul_in_ready) == (|norm_mul_in_ready)))
        else $fatal(0, "norm_mul_in_ready bits are not all uniform");
    assert ((!rst_ni || (&norm_mul_in_valid) == (|norm_mul_in_valid)))
        else $fatal(0, "norm_mul_in_valid bits are not all uniform");
    assert ((!rst_ni || (&norm_mul_out_ready) == (|norm_mul_out_ready)))
        else $fatal(0, "norm_mul_out_ready bits are not all uniform");
    assert ((!rst_ni || (&norm_mul_out_valid) == (|norm_mul_out_valid)))
        else $fatal(0, "norm_mul_out_valid bits are not all uniform");
end
`endif

for (genvar i_GEN = 0; i_GEN < VectorParallelism; i_GEN++) begin : parallel_norm_mul

    ternip_mul #(
        .InAPrecision(FixedPointPrecision),
        .InAExponent(FixedPointExponent),
        .InBPrecision(RmsValueReciprocalPrecision),
        .InBExponent(RmsValueReciprocalExponent),
        .OutPrecision(FixedPointPrecision),
        .OutExponent(FixedPointExponent)
    ) norm_mul (
        .clk_i,
        .rst_ni,

        .in_ready_o(norm_mul_in_ready[i_GEN]),
        .in_valid_i(norm_mul_in_valid[i_GEN]),
        .a_i(norm_mul_in_a[i_GEN]),
        .b_i(norm_mul_in_b),

        .out_ready_i(norm_mul_out_ready[i_GEN]),
        .out_valid_o(norm_mul_out_valid[i_GEN]),
        .y_o(norm_mul_out_result[i_GEN])
    );

end


// State Machine

always_comb begin
    in_ready_o = 0;
    rms_op_d = rms_op_q;
    in_vector1_select_d = in_vector1_select_q;
    in_vector2_select_d = in_vector2_select_q;
    in_rms_length_d = in_rms_length_q;
    state_d = state_q;

    vector_request_valid_o = 0;
    vector_request_write_not_read_o = 'x;
    vector_request_vector_addr_o = 'x;
    vector_request_vector_select_o = 'x;
    vector_request_w_data_o = 'x;
    vector_read_ready_o = 0;

    vector_request2_valid_o = 0;
    vector_request2_write_not_read_o = 'x;
    vector_request2_vector_addr_o = 'x;
    vector_request2_vector_select_o = 'x;
    vector_request2_w_data_o = 'x;

    vector_read_counter_d = vector_read_counter_q;
    vector_processed_counter_d = vector_processed_counter_q;

    accumulator_in_valid = 0;
    accumulator_in_final = 'x;
    accumulator_in_operand = 'x;
    accumulator_out_ready = 0;

    square_in_valid = '0;
    square_operand_in = 'x;
    square_out_ready = '0;

    div_in_valid = 0;
    div_in_dividend = 'x;
    div_in_divisor = 'x;

    rms_value_reciprocal_d = rms_value_reciprocal_q;
    rms_value_reciprocal_valid_d = rms_value_reciprocal_valid_q;

    norm_mul_in_valid = '0;
    norm_mul_in_a = 'x;
    norm_mul_in_b = 'x;
    norm_mul_out_ready = '0;
    norm_mul_out_result_buffer_d = norm_mul_out_result_buffer_q;
    norm_mul_out_result_buffer_valid_d = norm_mul_out_result_buffer_valid_q;

    rms_sqrt_out_ready = 0;

    if (state_q == WAITING_FOR_IN) begin
        in_ready_o = 1;
        if (in_ready_o && in_valid_i) begin
            rms_op_d = in_rms_op_i;
            in_vector1_select_d = in_vector1_select_i;
            in_vector2_select_d = in_vector2_select_i;
            in_rms_length_d = in_rms_length_i;
            if (in_rms_op_i == ternip_pkg::CLEAR) begin
                state_d = SENDING_FINAL_TO_ACCUMULATOR;
                rms_value_reciprocal_d = 'x;
                rms_value_reciprocal_valid_d = 0;
            end else if (in_rms_op_i == ternip_pkg::ACCUMULATE) begin
                state_d = WORKING;
                vector_read_counter_d = 0;
                vector_processed_counter_d = 0;
                rms_value_reciprocal_d = 'x;
                rms_value_reciprocal_valid_d = 0;
            end else if (in_rms_op_i == ternip_pkg::FINISH_ACCUMULATE) begin
                state_d = SENDING_FINAL_TO_ACCUMULATOR;
                rms_value_reciprocal_d = 'x;
                rms_value_reciprocal_valid_d = 0;
            end else if (in_rms_op_i == ternip_pkg::NORM) begin
                state_d = WORKING;
                vector_read_counter_d = 0;
                vector_processed_counter_d = 0;
            end
        end
    end else if ((state_q == SENDING_FINAL_TO_ACCUMULATOR) && (rms_op_q == ternip_pkg::CLEAR)) begin
        accumulator_in_valid = 1;
        accumulator_in_final = 1;
        accumulator_in_operand = 'x;
        accumulator_out_ready = 1; // drain any stale results
        if (accumulator_in_ready) begin
            state_d = WAITING_FOR_ACCUMULATOR;
        end
    end else if ((state_q == WAITING_FOR_ACCUMULATOR) && (rms_op_q == ternip_pkg::CLEAR)) begin
        accumulator_out_ready = 1;
        if (accumulator_out_valid) begin
            state_d = WAITING_FOR_IN;
            rms_op_d = ternip_pkg::NO_RMS_OP;
            in_vector1_select_d = 'x;
            in_vector2_select_d = 'x;
            in_rms_length_d  = 'x;
        end
    end else if ((state_q == WORKING) && (rms_op_q == ternip_pkg::ACCUMULATE)) begin
        if (vector_read_counter_q < NumChunksPerVector) begin // read request
            vector_request_valid_o = 1;
            vector_request_write_not_read_o = 0;
            vector_request_vector_addr_o = vector_read_counter_q;
            vector_request_vector_select_o = in_vector1_select_q;
            if (vector_request_ready_i) vector_read_counter_d++;
        end
        // read response -> square
        vector_read_ready_o = square_in_ready[0];
        square_in_valid = {VectorParallelism{ vector_read_valid_i }};
        square_operand_in = vector_read_data_i;
        // square -> accumulator
        square_out_ready = '1;
        accumulator_in_valid = square_out_valid[0];
        accumulator_in_final = 0;
        accumulator_in_operand = square_result_out;
        if (accumulator_in_ready && accumulator_in_valid) begin
            vector_processed_counter_d++;
            if (vector_processed_counter_q == NumChunksPerVector-1) begin
                state_d = WAITING_FOR_IN;
                rms_op_d = ternip_pkg::NO_RMS_OP;
                in_vector1_select_d = 'x;
                in_vector2_select_d = 'x;
                in_rms_length_d  = 'x;
                vector_read_counter_d = 'x;
                vector_processed_counter_d = 'x;
            end
        end
    end else if ((state_q == SENDING_FINAL_TO_ACCUMULATOR) && (rms_op_q == ternip_pkg::FINISH_ACCUMULATE)) begin
        accumulator_in_valid = 1;
        accumulator_in_final = 1;
        accumulator_in_operand = 0;
        accumulator_out_ready = 1;
        if (accumulator_in_ready) begin
            state_d = WAITING_FOR_ACCUMULATOR;
        end
    end else if ((state_q == WAITING_FOR_ACCUMULATOR) && (rms_op_q == ternip_pkg::FINISH_ACCUMULATE)) begin
        accumulator_out_ready = div_in_ready;
        if (accumulator_out_valid) begin
            state_d = WORKING;
            div_in_valid = 1;
            `ifndef SYNTHESIS
            assert(!rst_ni || $isunknown(in_rms_length_q) || (in_rms_length_q > 0));
            assert(!rst_ni || $isunknown(accumulator_out_result) || (accumulator_out_result >= 0)) else $fatal(0, "accumulator_out_result=%b", accumulator_out_result);
            `endif
            div_in_dividend = in_rms_length_q;
            div_in_divisor = (accumulator_out_result <= 0) ? 1 /* TODO */ : accumulator_out_result;
        end
    end else if ((state_q == WORKING) && (rms_op_q == ternip_pkg::FINISH_ACCUMULATE)) begin
        // wait for rms_value_reciprocal_divider -> ternip_sqrt -> valid
        rms_sqrt_out_ready = 1;
        if (rms_sqrt_out_valid) begin
            rms_value_reciprocal_d = rms_sqrt_y;
            rms_value_reciprocal_valid_d = 1;
            state_d = WAITING_FOR_IN;
            rms_op_d = ternip_pkg::NO_RMS_OP;
            in_vector1_select_d = 'x;
            in_vector2_select_d = 'x;
            in_rms_length_d  = 'x;
        end
    end else if ((state_q == WORKING) && (rms_op_q == ternip_pkg::NORM)) begin
        // read response -> multiplier
        vector_read_ready_o = norm_mul_in_ready[0];
        norm_mul_in_valid = {VectorParallelism{ vector_read_valid_i }};
        norm_mul_in_a = vector_read_data_i;
        norm_mul_in_b = rms_value_reciprocal_q;

        // Port 1: read request stream
        if (vector_read_counter_q < NumChunksPerVector) begin
            vector_request_valid_o = 1;
            vector_request_write_not_read_o = 0;
            vector_request_vector_addr_o = vector_read_counter_q;
            vector_request_vector_select_o = in_vector1_select_q;
            if (vector_request_ready_i) begin
                vector_read_counter_d++;
            end
        end

        // Port 2: write request stream (drains multiplier output buffer)
        if (norm_mul_out_result_buffer_valid_q) begin
            vector_request2_valid_o = 1;
            vector_request2_write_not_read_o = 1;
            vector_request2_vector_addr_o = vector_processed_counter_q;
            vector_request2_w_data_o = norm_mul_out_result_buffer_q;
            vector_request2_vector_select_o = in_vector2_select_q;
            if (vector_request2_ready_i) begin
                norm_mul_out_result_buffer_d = 'x;
                norm_mul_out_result_buffer_valid_d = 0;
                vector_processed_counter_d++;
                if (vector_processed_counter_q >= NumChunksPerVector-1) begin
                    state_d = WAITING_FOR_IN;
                    rms_op_d = ternip_pkg::NO_RMS_OP;
                    in_vector1_select_d = 'x;
                    in_vector2_select_d = 'x;
                    in_rms_length_d  = 'x;
                    vector_read_counter_d = 'x;
                    vector_processed_counter_d = 'x;
                end
            end
        end

        // multiplier -> buffer
        norm_mul_out_ready = {VectorParallelism{ !norm_mul_out_result_buffer_valid_q || vector_request2_ready_i }};
        if (norm_mul_out_ready[0] && norm_mul_out_valid[0]) begin
            norm_mul_out_result_buffer_d = norm_mul_out_result;
            norm_mul_out_result_buffer_valid_d = 1;
        end
    end
end

always_ff @(posedge clk_i) begin
    if (!rst_ni) begin
        state_q <= WAITING_FOR_IN;
        rms_value_reciprocal_valid_q <= 0;
        norm_mul_out_result_buffer_valid_q <= 0;
    end else begin
        state_q <= state_d;
        rms_value_reciprocal_valid_q <= rms_value_reciprocal_valid_d;
        norm_mul_out_result_buffer_valid_q <= norm_mul_out_result_buffer_valid_d;
    end
end
always_ff @(posedge clk_i) begin
    rms_op_q <= rms_op_d;
    vector_read_counter_q <= vector_read_counter_d;
    vector_processed_counter_q <= vector_processed_counter_d;

    in_vector1_select_q <= in_vector1_select_d;
    in_vector2_select_q <= in_vector2_select_d;
    in_rms_length_q <= in_rms_length_d;

    rms_value_reciprocal_q <= rms_value_reciprocal_d;
    norm_mul_out_result_buffer_q <= norm_mul_out_result_buffer_d;
    `ifndef SYNTHESIS
    if (!rst_ni) begin
        rms_op_q <= ternip_pkg::NO_RMS_OP;
        vector_read_counter_q <= 'x;
        vector_processed_counter_q <= 'x;

        in_vector1_select_q <= 'x;
        in_vector2_select_q <= 'x;
        in_rms_length_q <= 'x;

        rms_value_reciprocal_q <= 'x;
        norm_mul_out_result_buffer_q <= 'x;
    end
    `endif
end

endmodule
