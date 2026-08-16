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
// Each vector is stored as ternip_types#(Cfg)::NumChunksPerVector chunks. A request chooses one
// vector register and one chunk address, then either writes that chunk or starts
// a read for that chunk.
//
// Use request_ready_o/request_valid_i for both reads and writes. Reads return
// later on read_valid_o/read_data_o, along with the vector and chunk address
// that were read.

module ternip_vector_registers #(
    parameter ternip_pkg::ternip_cfg_t Cfg = `TERNIP_CFG
) (
    input  logic                               clk_i,
    input  logic                               rst_ni,

    output logic                               request_ready_o,
    input  logic                               request_valid_i,
    input  logic                               request_write_not_read_i,
    input  ternip_types#(Cfg)::vector_select_t request_vector_select_i,
    input  ternip_types#(Cfg)::vector_offset_t request_vector_addr_i,
    input  ternip_types#(Cfg)::vector_chunk_t  request_w_data_i,

    input  logic                               read_ready_i,
    output logic                               read_valid_o,
    output ternip_types#(Cfg)::vector_select_t read_vector_select_o,
    output ternip_types#(Cfg)::vector_offset_t read_addr_o,
    output ternip_types#(Cfg)::vector_chunk_t  read_data_o
);

logic [$bits(ternip_types#(Cfg)::vector_select_t)+$bits(ternip_types#(Cfg)::vector_offset_t)-1:0] request_mem_addr, read_mem_addr;
assign request_mem_addr = {request_vector_select_i, request_vector_addr_i};
assign {read_vector_select_o, read_addr_o} = read_mem_addr;

localparam int D_rounded_up = 2**$clog2(Cfg.D);

ternip_pipelined_mem #(
    .DATA_WIDTH(Cfg.FixedPointPrecision * Cfg.VectorParallelism),
    .NUM_ENTRIES(Cfg.NumVectorRegisters * D_rounded_up / Cfg.VectorParallelism),
    .DECOUPLED_READY(1)
) pipelined_mem (
    .clk_i,
    .rst_ni,

    .request_ready_o,
    .request_valid_i,
    .request_write_not_read_i,
    .request_addr_i(request_mem_addr),
    .request_w_data_i,

    .read_ready_i,
    .read_valid_o,
    .read_addr_o(read_mem_addr),
    .read_data_o
);

endmodule
