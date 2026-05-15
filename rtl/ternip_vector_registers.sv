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

// ternip_vector_registers
//
// Stores Ternip vector registers.
//
// Each vector is stored as NumChunksPerVector chunks. A request chooses one
// vector register and one chunk address, then either writes that chunk or starts
// a read for that chunk.
//
// The request2_* and read2_* interfaces are a second port to the same storage.
//
// Use request_ready_o/request_valid_i for both reads and writes. Reads return
// later on read_valid_o/read_data_o, along with the vector and chunk address
// that were read.

module ternip_vector_registers #(
    parameter int D                   = ternip_pkg::D,
    parameter int FixedPointPrecision = ternip_pkg::FixedPointPrecision,
    parameter int VectorParallelism   = ternip_pkg::VectorParallelism,
    parameter int NumVectorRegisters  = ternip_pkg::NumVectorRegisters,
    parameter int NumChunksPerVector  = ternip_pkg::NumChunksPerVector,

    localparam type fixed_point_t     = logic signed [ternip_pkg::FixedPointPrecision-1:0],
    localparam type vector_chunk_t    = fixed_point_t [VectorParallelism-1:0],
    localparam type vector_offset_t   = logic [$clog2(NumChunksPerVector)-1:0],
    localparam type vector_select_t   = logic [$clog2(NumVectorRegisters)-1:0]
) (
    input  logic           clk_i,
    input  logic           rst_ni,

    output logic           request_ready_o,
    input  logic           request_valid_i,
    input  logic           request_write_not_read_i,
    input  vector_select_t request_vector_select_i,
    input  vector_offset_t request_vector_addr_i,
    input  vector_chunk_t  request_w_data_i,

    input  logic           read_ready_i,
    output logic           read_valid_o,
    output vector_select_t read_vector_select_o,
    output vector_offset_t read_addr_o,
    output vector_chunk_t  read_data_o,

    output logic           request2_ready_o,
    input  logic           request2_valid_i,
    input  logic           request2_write_not_read_i,
    input  vector_select_t request2_vector_select_i,
    input  vector_offset_t request2_vector_addr_i,
    input  vector_chunk_t  request2_w_data_i,

    input  logic           read2_ready_i,
    output logic           read2_valid_o,
    output vector_select_t read2_vector_select_o,
    output vector_offset_t read2_addr_o,
    output vector_chunk_t  read2_data_o
);

logic [$bits(vector_select_t)+$bits(vector_offset_t)-1:0] request_mem_addr, read_mem_addr;
logic [$bits(vector_select_t)+$bits(vector_offset_t)-1:0] request2_mem_addr, read2_mem_addr;
assign request_mem_addr = {request_vector_select_i, request_vector_addr_i};
assign request2_mem_addr = {request2_vector_select_i, request2_vector_addr_i};
assign {read_vector_select_o, read_addr_o} = read_mem_addr;
assign {read2_vector_select_o, read2_addr_o} = read2_mem_addr;

localparam int D_rounded_up = 2**$clog2(D);

ternip_dual_port_mem #(
    .DATA_WIDTH($bits(read_data_o)),
    .NUM_ENTRIES(NumVectorRegisters * D_rounded_up / VectorParallelism),
    .UNCOUPLED_READY(1)
) dual_port_mem (
    .clk_i,
    .rst_ni,

    .a_request_ready_o(request_ready_o),
    .a_request_valid_i(request_valid_i),
    .a_request_write_not_read_i(request_write_not_read_i),
    .a_request_addr_i(request_mem_addr),
    .a_request_w_data_i(request_w_data_i),

    .a_read_ready_i(read_ready_i),
    .a_read_valid_o(read_valid_o),
    .a_read_addr_o(read_mem_addr),
    .a_read_data_o(read_data_o),

    .b_request_ready_o(request2_ready_o),
    .b_request_valid_i(request2_valid_i),
    .b_request_write_not_read_i(request2_write_not_read_i),
    .b_request_addr_i(request2_mem_addr),
    .b_request_w_data_i(request2_w_data_i),

    .b_read_ready_i(read2_ready_i),
    .b_read_valid_o(read2_valid_o),
    .b_read_addr_o(read2_mem_addr),
    .b_read_data_o(read2_data_o)
);

endmodule
