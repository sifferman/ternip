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

// ternip_rowwise_operation
//
// Functional unit for elementwise vector operations.
//
// This unit reads one or more vector registers, applies the selected rowwise_op_e
// to each lane, and writes the result vector. ADD/SUB are combinational per
// chunk. MUL, DIV, SIG, CSIG, and SILU route chunks through the corresponding
// ready/valid math blocks and buffer multicycle results before writing them
// back.
//
// Use in_vector1_r_select_i as the first source, in_vector2_r_select_i as the
// second source for binary operations, and in_vector3_r_select_i as the
// destination register. The vector register request/read channels must be
// connected to ternip_vector_registers.

module ternip_rowwise_operation #(
    parameter int D                   = ternip_pkg::D,
    parameter int FixedPointPrecision = ternip_pkg::FixedPointPrecision,
    parameter int FixedPointExponent  = ternip_pkg::FixedPointExponent,
    parameter int VectorParallelism   = ternip_pkg::VectorParallelism,
    parameter int LutParallelism      = ternip_pkg::LutParallelism,
    parameter int NumVectorRegisters  = ternip_pkg::NumVectorRegisters,
    parameter int NumChunksPerVector  = ternip_pkg::NumChunksPerVector,
    parameter bit UseHardSigmoid      = ternip_pkg::UseHardSigmoid,

    parameter ternip_pkg::mul_impl_e MultiplicationImplementation = ternip_pkg::MultiplicationImplementation,
    // parameter ternip_pkg::div_impl_e DivisionImplementation       = ternip_pkg::DivisionImplementation,

    localparam type fixed_point_t   = logic signed [ternip_pkg::FixedPointPrecision-1:0],
    localparam type vector_chunk_t  = fixed_point_t [VectorParallelism-1:0],
    localparam type vector_offset_t = logic [$clog2(NumChunksPerVector)-1:0],
    localparam type vector_select_t = logic [$clog2(NumVectorRegisters)-1:0]
) (
    input  logic                    clk_i,
    input  logic                    rst_ni,

    output logic                    in_ready_o,
    input  logic                    in_valid_i,
    input  ternip_pkg::rowwise_op_e in_operation_i,
    input  vector_select_t          in_vector1_r_select_i,
    input  vector_select_t          in_vector2_r_select_i,
    input  vector_select_t          in_vector3_r_select_i,

    input  logic           vector_request_ready_i,
    output logic           vector_request_valid_o,
    output logic           vector_request_write_not_read_o,
    output vector_select_t vector_request_vector_select_o,
    output vector_offset_t vector_request_vector_addr_o,
    output vector_chunk_t  vector_request_w_data_o,

    output logic           vector_read_ready_o,
    input  logic           vector_read_valid_i,
    input  vector_chunk_t  vector_read_data_i
);

logic [$clog2(D+1):0] read_request_counter_d, read_request_counter_q;
logic [$clog2(D+1):0] read_response_counter_d, read_response_counter_q;
logic [$clog2(D+1):0] write_request_counter_d, write_request_counter_q;
ternip_pkg::rowwise_op_e vector_operation_d, vector_operation_q;

wire operation_is_multicycle = vector_operation_q inside {ternip_pkg::MUL, ternip_pkg::DIV, ternip_pkg::SIG, ternip_pkg::CSIG, ternip_pkg::SILU};

vector_select_t vector1_select_d, vector1_select_q;
vector_select_t vector2_select_d, vector2_select_q;
vector_select_t vector3_select_d, vector3_select_q;

wire operation_is_multioperand = vector_operation_q inside {ternip_pkg::ADD, ternip_pkg::SUB, ternip_pkg::MUL, ternip_pkg::DIV};
vector_chunk_t vector1_r_data_d, vector1_r_data_q;

vector_chunk_t rowwise_add_result;
vector_chunk_t rowwise_sub_result;

logic [VectorParallelism-1:0] rowwise_mul_in_ready;
logic [VectorParallelism-1:0] rowwise_mul_in_valid;
logic [VectorParallelism-1:0] rowwise_mul_out_ready;
logic [VectorParallelism-1:0] rowwise_mul_out_valid;
vector_chunk_t rowwise_mul_result;

logic [VectorParallelism-1:0] rowwise_div_in_ready;
logic [VectorParallelism-1:0] rowwise_div_in_valid;
logic [VectorParallelism-1:0] rowwise_div_out_ready;
logic [VectorParallelism-1:0] rowwise_div_out_valid;
vector_chunk_t rowwise_div_result;

logic all_mul_in_ready, all_div_in_ready;
logic all_mul_out_valid, all_div_out_valid;
assign all_mul_in_ready = &rowwise_mul_in_ready;
assign all_mul_out_valid = &rowwise_mul_out_valid;
assign all_div_in_ready = &rowwise_div_in_ready;
assign all_div_out_valid = &rowwise_div_out_valid;

logic rowwise_sig_in_ready;
logic rowwise_sig_in_valid;
logic rowwise_sig_out_ready;
logic rowwise_sig_out_valid;
vector_chunk_t rowwise_sig_result;

logic rowwise_csig_in_ready;
logic rowwise_csig_in_valid;
logic rowwise_csig_out_ready;
logic rowwise_csig_out_valid;
vector_chunk_t rowwise_csig_result;
vector_chunk_t rowwise_csig_out_result;

logic rowwise_silu_in_ready;
logic rowwise_silu_in_valid;
logic rowwise_silu_out_ready;
logic rowwise_silu_out_valid;
vector_chunk_t rowwise_silu_result;
vector_chunk_t rowwise_silu_out_result;

logic multicycle_in_ready;
logic multicycle_in_valid;
logic multicycle_out_ready;
logic multicycle_out_valid;
logic multicycle_result_buffer_valid_d, multicycle_result_buffer_valid_q;
vector_chunk_t multicycle_result_buffer_d, multicycle_result_buffer_q;

enum logic [2:0] {
    WAITING_FOR_IN,
    WORKING
} state_d, state_q = WAITING_FOR_IN; // for assertions at time=0

always_comb begin
    in_ready_o = 0;

    read_request_counter_d = read_request_counter_q;
    read_response_counter_d = read_response_counter_q;
    write_request_counter_d = write_request_counter_q;
    state_d = state_q;
    vector_operation_d = vector_operation_q;
    vector1_select_d = vector1_select_q;
    vector2_select_d = vector2_select_q;
    vector3_select_d = vector3_select_q;

    rowwise_mul_in_valid = '0;
    rowwise_mul_out_ready = '0;
    rowwise_div_in_valid = '0;
    rowwise_div_out_ready = '0;

    vector_request_valid_o = 0;
    vector_request_write_not_read_o = 0;
    vector_request_vector_select_o = 0;
    vector_request_vector_addr_o = 'x;

    vector_read_ready_o = 0;
    vector1_r_data_d = vector1_r_data_q;

    rowwise_sig_in_valid = 0;
    rowwise_sig_out_ready = 0;
    rowwise_csig_in_valid = 0;
    rowwise_csig_out_ready = 0;
    rowwise_silu_in_valid = 0;
    rowwise_silu_out_ready = 0;

    multicycle_in_ready = 0;
    multicycle_in_valid = 0;
    multicycle_out_ready = 0;
    multicycle_out_valid = 0;
    multicycle_result_buffer_d = multicycle_result_buffer_q;
    multicycle_result_buffer_valid_d = multicycle_result_buffer_valid_q;

    if (state_q == WAITING_FOR_IN) begin
        in_ready_o = 1;
        if (in_valid_i) begin
            read_request_counter_d = 0;
            read_response_counter_d = 0;
            write_request_counter_d = 0;
            vector_operation_d = in_operation_i;
            vector1_select_d = in_vector1_r_select_i;
            vector2_select_d = in_vector2_r_select_i;
            vector3_select_d = in_vector3_r_select_i;
            state_d = WORKING;
        end
    end else if (state_q == WORKING) begin
        if (!operation_is_multioperand && !operation_is_multicycle) begin
            state_d = WORKING;
        end else if (!operation_is_multioperand && operation_is_multicycle) begin
            // SIG, CSIG, SILU
            case (vector_operation_q)
                ternip_pkg::SIG:  multicycle_in_ready = rowwise_sig_in_ready;
                ternip_pkg::CSIG: multicycle_in_ready = rowwise_csig_in_ready;
                ternip_pkg::SILU: multicycle_in_ready = rowwise_silu_in_ready;
            endcase
            vector_read_ready_o = multicycle_in_ready;

            multicycle_in_valid = vector_read_valid_i;
            case (vector_operation_q)
                ternip_pkg::SIG:  rowwise_sig_in_valid = multicycle_in_valid;
                ternip_pkg::CSIG: rowwise_csig_in_valid = multicycle_in_valid;
                ternip_pkg::SILU: rowwise_silu_in_valid = multicycle_in_valid;
            endcase

            // buffer -> write request
            if (multicycle_result_buffer_valid_q) begin
                vector_request_valid_o = 1;
                vector_request_write_not_read_o = 1;
                vector_request_vector_select_o = vector3_select_q;
                vector_request_vector_addr_o = write_request_counter_q;
                if (vector_request_ready_i) begin
                    multicycle_result_buffer_d = 'x;
                    multicycle_result_buffer_valid_d = 0;
                    write_request_counter_d++;
                    if (write_request_counter_q >= NumChunksPerVector-1) begin
                        state_d = WAITING_FOR_IN;
                    end
                end
            end else if (read_request_counter_q < NumChunksPerVector) begin // read request
                // if a read was just received, do not do another read
                vector_request_valid_o = 1;
                vector_request_write_not_read_o = 0;
                vector_request_vector_select_o = vector1_select_q;
                vector_request_vector_addr_o = read_request_counter_q;
                if (vector_request_ready_i && vector_request_valid_o) begin
                    read_request_counter_d++;
                end
            end

            // sig, csig, silu -> buffer
            multicycle_out_ready = !multicycle_result_buffer_valid_q || vector_request_ready_i;
            case (vector_operation_q)
                ternip_pkg::SIG:  multicycle_out_valid = rowwise_sig_out_valid;
                ternip_pkg::CSIG: multicycle_out_valid = rowwise_csig_out_valid;
                ternip_pkg::SILU: multicycle_out_valid = rowwise_silu_out_valid;
            endcase

            case (vector_operation_q)
                ternip_pkg::SIG:  rowwise_sig_out_ready  = multicycle_out_ready;
                ternip_pkg::CSIG: rowwise_csig_out_ready = multicycle_out_ready;
                ternip_pkg::SILU: rowwise_silu_out_ready = multicycle_out_ready;
            endcase

            if (multicycle_out_ready && multicycle_out_valid) begin
                case (vector_operation_q)
                    ternip_pkg::SIG:  multicycle_result_buffer_d = rowwise_sig_result;
                    ternip_pkg::CSIG: multicycle_result_buffer_d = rowwise_csig_result;
                    ternip_pkg::SILU: multicycle_result_buffer_d = rowwise_silu_result;
                endcase
                multicycle_result_buffer_valid_d = 1;
            end

        end else if (operation_is_multioperand && !operation_is_multicycle) begin
            // ADD / SUB
            vector_read_ready_o = 1;
            if (vector_read_valid_i) begin
                read_response_counter_d++;
                if (read_response_counter_q % 2 == 0) begin // received vec1 read
                    vector1_r_data_d = vector_read_data_i;
                end
            end

            // Request issuance: write takes priority over read on the cycle
            // an odd response arrives; otherwise issue the next read.
            if (vector_read_valid_i && (read_response_counter_q % 2 == 1)) begin // received vec2 read -> write
                vector_request_valid_o = 1;
                vector_request_write_not_read_o = 1;
                vector_request_vector_select_o = vector3_select_q;
                vector_request_vector_addr_o = write_request_counter_q;
                if (vector_request_ready_i) begin
                    write_request_counter_d++;
                    if (write_request_counter_q >= NumChunksPerVector-1) begin
                        state_d = WAITING_FOR_IN;
                    end
                end
            end else if (read_request_counter_q < 2*NumChunksPerVector) begin
                vector_request_valid_o = 1;
                vector_request_write_not_read_o = 0;
                if (read_request_counter_q % 2 == 0)
                    vector_request_vector_select_o = vector1_select_q;
                else
                    vector_request_vector_select_o = vector2_select_q;
                vector_request_vector_addr_o = read_request_counter_q / 2;
                if (vector_request_ready_i) begin
                    read_request_counter_d++;
                end
            end
        end else if (operation_is_multioperand && operation_is_multicycle) begin
            // MUL / DIV

            // send read response to multioperand module
            case (vector_operation_q)
                ternip_pkg::MUL: multicycle_in_ready = all_mul_in_ready;
                ternip_pkg::DIV: multicycle_in_ready = all_mul_in_ready;
            endcase
            if (read_response_counter_q < 2*NumChunksPerVector) begin
                vector_read_ready_o = (read_response_counter_q % 2 == 0) || multicycle_in_ready;
                if (vector_read_ready_o && vector_read_valid_i) begin
                    read_response_counter_d++;
                    if (read_response_counter_q % 2 == 0) begin // received vec1 read
                        vector1_r_data_d = vector_read_data_i;
                    end else begin // received vec2 read
                        multicycle_in_valid = 1;
                    end
                end
            end
            case (vector_operation_q)
                ternip_pkg::MUL: for (int i = 0; i < VectorParallelism; i++) rowwise_mul_in_valid[i] = multicycle_in_valid;
                ternip_pkg::DIV: for (int i = 0; i < VectorParallelism; i++) rowwise_div_in_valid[i] = multicycle_in_valid;
            endcase

            // buffer -> write request
            if (multicycle_result_buffer_valid_q) begin
                vector_request_valid_o = 1;
                vector_request_write_not_read_o = 1;
                vector_request_vector_select_o = vector3_select_q;
                vector_request_vector_addr_o = write_request_counter_q;
                if (vector_request_ready_i) begin
                    multicycle_result_buffer_d = 'x;
                    multicycle_result_buffer_valid_d = 0;
                    write_request_counter_d++;
                    if (write_request_counter_q >= NumChunksPerVector-1) begin
                        state_d = WAITING_FOR_IN;
                    end
                end
            end else if (read_request_counter_q < 2*NumChunksPerVector) begin // read request
                vector_request_valid_o = 1;
                vector_request_write_not_read_o = 0;
                if (read_request_counter_q % 2 == 0) begin // request vec1 read
                    vector_request_vector_select_o = vector1_select_q;
                end else begin // request vec2 read
                    vector_request_vector_select_o = vector2_select_q;
                end
                vector_request_vector_addr_o = read_request_counter_q / 2;
                if (vector_request_ready_i) begin
                    read_request_counter_d++;
                end
            end

            // multioperand output -> buffer
            multicycle_out_ready = !multicycle_result_buffer_valid_q || vector_request_ready_i;
            if (vector_operation_q == ternip_pkg::MUL) begin
                for (int i = 0; i < VectorParallelism; i++) rowwise_mul_out_ready[i] = multicycle_out_ready;
                multicycle_out_valid = all_mul_out_valid;
            end else if (vector_operation_q == ternip_pkg::DIV) begin
                for (int i = 0; i < VectorParallelism; i++) rowwise_div_out_ready[i] = multicycle_out_ready;
                multicycle_out_valid = all_div_out_valid;
            end

            if (multicycle_out_ready && multicycle_out_valid) begin
                case (vector_operation_q)
                    ternip_pkg::MUL: multicycle_result_buffer_d = rowwise_mul_result;
                    ternip_pkg::DIV: multicycle_result_buffer_d = rowwise_div_result;
                endcase
                multicycle_result_buffer_valid_d = 1;
            end
        end
    end
end

always_ff @(posedge clk_i) begin
    if (!rst_ni) begin
        state_q <= WAITING_FOR_IN;
        multicycle_result_buffer_valid_q <= 0;
    end else begin
        state_q <= state_d;
        multicycle_result_buffer_valid_q <= multicycle_result_buffer_valid_d;
    end
end
always_ff @(posedge clk_i) begin
    read_request_counter_q <= read_request_counter_d;
    read_response_counter_q <= read_response_counter_d;
    write_request_counter_q <= write_request_counter_d;

    vector_operation_q <= vector_operation_d;
    vector1_r_data_q <= vector1_r_data_d;

    vector1_select_q <= vector1_select_d;
    vector2_select_q <= vector2_select_d;
    vector3_select_q <= vector3_select_d;

    multicycle_result_buffer_q <= multicycle_result_buffer_d;
    `ifndef SYNTHESIS
    if (!rst_ni) begin
        read_request_counter_q <= 'x;
        read_response_counter_q <= 'x;
        write_request_counter_q <= 'x;

        vector_operation_q <= ternip_pkg::NOP;
        vector1_r_data_q <= 'x;

        vector1_select_q <= 'x;
        vector2_select_q <= 'x;
        vector3_select_q <= 'x;

        multicycle_result_buffer_q <= 'x;
    end
    `endif
end

for (genvar i_GEN = 0; i_GEN < VectorParallelism; i_GEN++) begin

    ternip_add #(
        .FixedPointPrecision(FixedPointPrecision)
    ) add (
        .a_i(vector1_r_data_q[i_GEN]),
        .b_i(vector_read_data_i[i_GEN]),
        .y_o(rowwise_add_result[i_GEN])
    );

    ternip_sub #(
        .FixedPointPrecision(FixedPointPrecision)
    ) sub (
        .a_i(vector1_r_data_q[i_GEN]),
        .b_i(vector_read_data_i[i_GEN]),
        .y_o(rowwise_sub_result[i_GEN])
    );

    ternip_mul #(
        .InAPrecision(FixedPointPrecision),
        .InAExponent(FixedPointExponent),
        .InBPrecision(FixedPointPrecision),
        .InBExponent(FixedPointExponent),
        .OutPrecision(FixedPointPrecision),
        .OutExponent(FixedPointExponent),
        .Implementation(MultiplicationImplementation)
    ) mul (
        .clk_i,
        .rst_ni,
        .in_ready_o(rowwise_mul_in_ready[i_GEN]),
        .in_valid_i(rowwise_mul_in_valid[i_GEN]),
        .a_i(vector1_r_data_q[i_GEN]),
        .b_i(vector_read_data_i[i_GEN]),

        .out_ready_i(rowwise_mul_out_ready[i_GEN]),
        .out_valid_o(rowwise_mul_out_valid[i_GEN]),
        .y_o(rowwise_mul_result[i_GEN])
    );

    ternip_div #(
        .InAPrecision(FixedPointPrecision),
        .InAExponent(FixedPointExponent),
        .InBPrecision(FixedPointPrecision),
        .InBExponent(FixedPointExponent),
        .OutPrecision(FixedPointPrecision),
        .OutExponent(FixedPointExponent),
        .Implementation(ternip_pkg::DIV_NONE) // Disable division unit
    ) div (
        .clk_i,
        .rst_ni,
        .in_ready_o(rowwise_div_in_ready[i_GEN]),
        .in_valid_i(rowwise_div_in_valid[i_GEN]),
        .a_i(vector1_r_data_q[i_GEN]),
        .b_i(vector_read_data_i[i_GEN]),

        .out_ready_i(rowwise_div_out_ready[i_GEN]),
        .out_valid_o(rowwise_div_out_valid[i_GEN]),
        .y_o(rowwise_div_result[i_GEN])
    );

end

ternip_sig_parallelized #(
    .FixedPointPrecision(FixedPointPrecision),
    .FixedPointExponent(FixedPointExponent),
    .VectorParallelism(VectorParallelism),
    .LutParallelism(LutParallelism),
    .UseHardSigmoid(UseHardSigmoid)
) sig_parallelized (
    .clk_i,
    .rst_ni,

    .in_ready_o(rowwise_sig_in_ready),
    .in_valid_i(rowwise_sig_in_valid),
    .vector_data_i(vector_read_data_i),

    .out_ready_i(rowwise_sig_out_ready),
    .out_valid_o(rowwise_sig_out_valid),
    .vector_data_o(rowwise_sig_result)
);

ternip_csig_parallelized #(
    .FixedPointPrecision(FixedPointPrecision),
    .FixedPointExponent(FixedPointExponent),
    .VectorParallelism(VectorParallelism),
    .LutParallelism(LutParallelism),
    .UseHardSigmoid(UseHardSigmoid)
) csig_parallelized (
    .clk_i,
    .rst_ni,

    .in_ready_o(rowwise_csig_in_ready),
    .in_valid_i(rowwise_csig_in_valid),
    .vector_data_i(vector_read_data_i),

    .out_ready_i(rowwise_csig_out_ready),
    .out_valid_o(rowwise_csig_out_valid),
    .vector_data_o(rowwise_csig_result)
);

ternip_silu_parallelized #(
    .FixedPointPrecision(FixedPointPrecision),
    .FixedPointExponent(FixedPointExponent),
    .VectorParallelism(VectorParallelism),
    .LutParallelism(LutParallelism),
    .UseHardSigmoid(UseHardSigmoid)
) silu_parallelized (
    .clk_i,
    .rst_ni,

    .in_ready_o(rowwise_silu_in_ready),
    .in_valid_i(rowwise_silu_in_valid),
    .vector_data_i(vector_read_data_i),

    .out_ready_i(rowwise_silu_out_ready),
    .out_valid_o(rowwise_silu_out_valid),
    .vector_data_o(rowwise_silu_result)
);

always_comb begin
    unique case (vector_operation_q)
        ternip_pkg::ADD:     vector_request_w_data_o = rowwise_add_result;
        ternip_pkg::SUB:     vector_request_w_data_o = rowwise_sub_result;
        ternip_pkg::MUL:     vector_request_w_data_o = multicycle_result_buffer_q;
        ternip_pkg::DIV:     vector_request_w_data_o = multicycle_result_buffer_q;
        ternip_pkg::SIG:     vector_request_w_data_o = multicycle_result_buffer_q;
        ternip_pkg::CSIG:    vector_request_w_data_o = multicycle_result_buffer_q;
        ternip_pkg::SILU:    vector_request_w_data_o = multicycle_result_buffer_q;
        default: vector_request_w_data_o = 'x;
    endcase
end

endmodule
