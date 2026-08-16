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

// ternip_loadstore
//
// Functional unit for loading and storing vector registers through a DDR stream
// interface.
//
// LDV starts a DDR read stream and writes each incoming vector chunk into the
// selected vector register. SV reads chunks from the selected vector register
// and streams them to DDR. The instruction-side in_* handshake accepts one
// load/store command; the unit then sequences vector-memory and DDR handshakes
// until the whole vector transfer completes.
//
// Use in_vector_memory_address_i as the stream base address and
// in_vector_select_i as the target/source vector register. The emitted stream
// length is Cfg.BatchSize * VectorSizeInBytes.

module ternip_loadstore #(
    parameter ternip_pkg::ternip_cfg_t Cfg = `TERNIP_CFG
) (
    input  logic                               clk_i,
    input  logic                               rst_ni,

    output logic                               in_ready_o,
    input  logic                               in_valid_i,
    input  ternip_pkg::loadstore_op_e          in_vector_operation_i,
    input  ternip_types#(Cfg)::ddr_address_t   in_vector_memory_address_i,
    input  ternip_types#(Cfg)::vector_select_t in_vector_select_i,

    // vector request interface
    input  logic                               vector_request_ready_i,
    output logic                               vector_request_valid_o,
    output logic                               vector_request_write_not_read_o,
    output ternip_types#(Cfg)::vector_select_t vector_request_vector_select_o,
    output ternip_types#(Cfg)::vector_offset_t vector_request_vector_addr_o,
    output ternip_types#(Cfg)::vector_chunk_t  vector_request_w_data_o,

    // vector read interface
    output logic                               vector_read_ready_o,
    input  logic                               vector_read_valid_i,
    input  ternip_types#(Cfg)::vector_offset_t vector_read_addr_i,
    input  ternip_types#(Cfg)::vector_chunk_t  vector_read_data_i,

    // ddr stream start config
    input  logic                               ddr_stream_ready_i,
    output logic                               ddr_stream_valid_o,
    output ternip_types#(Cfg)::ddr_address_t   ddr_stream_address_o,
    output logic                               ddr_stream_write_not_read_o,
    output logic [31:0]                        ddr_stream_length_o,

    // read data is streamed in sequentially
    output logic                               ddr_r_ready_o,
    input  logic                               ddr_r_valid_i,
    input  ternip_types#(Cfg)::vector_chunk_t  ddr_r_data_i,

    // write data should be streamed out sequentially
    input  logic                               ddr_w_ready_i,
    output logic                               ddr_w_valid_o,
    output ternip_types#(Cfg)::vector_chunk_t  ddr_w_data_o,

    output logic [63:0]                        debug_o
);

localparam int VectorSizeInBytes = Cfg.D * Cfg.FixedPointPrecision / 8;

enum logic [1:0] {
    WAITING_FOR_IN,
    WAITING_FOR_DDR_STREAM_READY,
    WORKING
} state_d, state_q = WAITING_FOR_IN; // for assertions at time=0

logic [$clog2(Cfg.D):0] vector_counter_d, vector_counter_q;
ternip_pkg::loadstore_op_e vector_operation_d, vector_operation_q;
ternip_types#(Cfg)::vector_select_t vector_select_d, vector_select_q;
ternip_types#(Cfg)::ddr_address_t vector_memory_address_d, vector_memory_address_q;

always_ff @(posedge clk_i) begin
    debug_o[0+:4] <= state_q;
    debug_o[4+:4] <= vector_operation_q;
    debug_o[8+:4] <= vector_request_ready_i;
    debug_o[12+:4] <= vector_request_valid_o;
    debug_o[16+:4] <= vector_read_ready_o;
    debug_o[20+:4] <= vector_read_valid_i;
    debug_o[24+:4] <= ddr_stream_ready_i;
    debug_o[28+:4] <= ddr_stream_valid_o;
    debug_o[32+:4] <= ddr_r_ready_o;
    debug_o[36+:4] <= ddr_r_valid_i;
    debug_o[40+:4] <= ddr_w_ready_i;
    debug_o[44+:4] <= ddr_w_valid_o;
    debug_o[48+:4] <= 0;
    debug_o[52+:4] <= 0;
    debug_o[56+:4] <= 0;
    debug_o[60+:4] <= 4'hf;
end

assign vector_request_vector_select_o = vector_select_q;
assign ddr_stream_length_o = Cfg.BatchSize * VectorSizeInBytes;

always_comb begin
    state_d = state_q;
    in_ready_o = 0;

    vector_counter_d = vector_counter_q;
    vector_select_d = vector_select_q;
    vector_operation_d = vector_operation_q;
    vector_memory_address_d = vector_memory_address_q;

    vector_request_valid_o = 0;
    vector_request_write_not_read_o = 'x;
    vector_request_vector_addr_o = 'x;
    vector_request_w_data_o = 'x;
    vector_read_ready_o = 0;

    ddr_stream_address_o = 'x;
    ddr_w_data_o = 'x;
    ddr_w_valid_o = 0;
    ddr_r_ready_o = 0;
    ddr_stream_write_not_read_o = 'x;

    ddr_stream_valid_o = 0;

    if (state_q == WAITING_FOR_IN) begin
        in_ready_o = 1;
        if (in_valid_i) begin
            state_d = WAITING_FOR_DDR_STREAM_READY;
            vector_select_d = in_vector_select_i;
            vector_operation_d = in_vector_operation_i;
            vector_memory_address_d = in_vector_memory_address_i;
        end
    end else if (state_q == WAITING_FOR_DDR_STREAM_READY) begin
        vector_counter_d = 0;
        ddr_stream_valid_o = 1;
        ddr_stream_address_o = vector_memory_address_q;
        ddr_stream_write_not_read_o = (vector_operation_q == ternip_pkg::SV);
        if (ddr_stream_ready_i)
            state_d = WORKING;
    end else if ((state_q == WORKING)&&(vector_operation_q == ternip_pkg::LDV)) begin
        // read from ddr and write to vector
        ddr_r_ready_o = vector_request_ready_i;
        if (ddr_r_ready_o && ddr_r_valid_i) begin
            vector_request_valid_o = 1;
            vector_request_write_not_read_o = 1;
            vector_request_vector_addr_o = vector_counter_q;
            vector_request_w_data_o = ddr_r_data_i;
            vector_counter_d++;
            if (vector_counter_q == ternip_types#(Cfg)::NumChunksPerVector-1) begin
                state_d = WAITING_FOR_IN;
            end
        end
    end else if ((state_q == WORKING)&&(vector_operation_q == ternip_pkg::SV)) begin
        // read from vector
        if (vector_counter_q != ternip_types#(Cfg)::NumChunksPerVector) begin
            vector_request_valid_o = 1;
            vector_request_write_not_read_o = 0;
            vector_request_vector_addr_o = vector_counter_q;
            if (vector_request_ready_i) vector_counter_d++;
        end
        // write to ddr
        vector_read_ready_o = ddr_w_ready_i;
        ddr_w_valid_o = vector_read_valid_i;
        ddr_w_data_o = vector_read_data_i;
        if (ddr_w_ready_i && ddr_w_valid_o) begin
            if (vector_read_addr_i == ternip_types#(Cfg)::NumChunksPerVector-1) state_d = WAITING_FOR_IN;
        end
    end
end

always_ff @(posedge clk_i) begin
    if (!rst_ni) begin
        state_q <= WAITING_FOR_IN;
    end else begin
        state_q <= state_d;
    end
end
always_ff @(posedge clk_i) begin
    vector_counter_q <= vector_counter_d;
    vector_operation_q <= vector_operation_d;

    vector_select_q <= vector_select_d;
    vector_memory_address_q <= vector_memory_address_d;
    `ifndef SYNTHESIS
    if (!rst_ni) begin
        vector_counter_q <= 'x;
        vector_operation_q <= ternip_pkg::NO_LS_OP;

        vector_select_q <= 'x;
        vector_memory_address_q <= 'x;
    end
    `endif
end

endmodule
