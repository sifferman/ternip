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

// ternip_sig_parallelized
//
// Vector fixed-point sigmoid.
//
// Applies ternip_sig (sigmoid) to every lane of one vector_chunk_t.
// Hard-sigmoid mode processes VectorParallelism lanes at a time.
// LUT mode processes LutParallelism lanes at a time.

module ternip_sig_parallelized #(
    parameter ternip_pkg::ternip_cfg_t Cfg = `TERNIP_CFG,

    parameter int FixedPointPrecision = Cfg.FixedPointPrecision,
    parameter int FixedPointExponent  = Cfg.FixedPointExponent,
    parameter int VectorParallelism   = Cfg.VectorParallelism,
    parameter int LutParallelism      = Cfg.LutParallelism,
    parameter bit UseHardSigmoid      = Cfg.UseHardSigmoid,

    localparam type fixed_point_t  = logic signed [FixedPointPrecision-1:0],
    localparam type vector_chunk_t = fixed_point_t [VectorParallelism-1:0]
) (
    input  logic          clk_i,
    input  logic          rst_ni,

    output logic          in_ready_o,
    input  logic          in_valid_i,
    input  vector_chunk_t vector_data_i,

    input  logic          out_ready_i,
    output logic          out_valid_o,
    output vector_chunk_t vector_data_o
);

localparam int Parallelism = UseHardSigmoid ? VectorParallelism : LutParallelism;

logic                           r_in_ready;
logic                           r_in_valid;
vector_chunk_t                  r_in_data;
logic                           r_out_ready;
logic                           r_out_valid;
fixed_point_t [Parallelism-1:0] r_out_data;

logic                           w_in_ready;
logic                           w_in_valid;
fixed_point_t [Parallelism-1:0] w_in_data;
logic                           w_out_ready;
logic                           w_out_valid;
vector_chunk_t                  w_out_data;

assign in_ready_o = r_in_ready;
assign r_in_valid = in_valid_i;
assign r_in_data = vector_data_i;

// https://github.com/bespoke-silicon-group/basejump_stl/blob/a43571d2/bsg_dataflow/bsg_parallel_in_serial_out.sv
bsg_parallel_in_serial_out #(
    .width_p(Parallelism*FixedPointPrecision),
    .els_p(VectorParallelism/Parallelism),
    .hi_to_lo_p(0)
) piso_loadstore_r (
    .clk_i,
    .reset_i(!rst_ni),

    .valid_i(r_in_valid),
    .data_i(r_in_data),
    .ready_and_o(r_in_ready),

    .valid_o(r_out_valid),
    .data_o(r_out_data),
    .yumi_i(r_out_ready && r_out_valid)
);

for (genvar i_GEN = 0; i_GEN < Parallelism; i_GEN++) begin
    ternip_sig #(
        .FixedPointPrecision(FixedPointPrecision),
        .FixedPointExponent(FixedPointExponent),
        .UseHardSigmoid(UseHardSigmoid)
    ) sig (
        .a_i(r_out_data[i_GEN]),
        .y_o(w_in_data[i_GEN])
    );
end

assign r_out_ready = w_in_ready;
assign w_in_valid  = r_out_valid;

// https://github.com/bespoke-silicon-group/basejump_stl/blob/a43571d2/bsg_dataflow/bsg_serial_in_parallel_out_full.sv
bsg_serial_in_parallel_out_full #(
    .width_p(Parallelism*FixedPointPrecision),
    .els_p(VectorParallelism/Parallelism),
    .hi_to_lo_p(0)
) sipo_loadstore_w (
    .clk_i,
    .reset_i(!rst_ni),

    .ready_and_o(w_in_ready),
    .v_i(w_in_valid),
    .data_i(w_in_data),

    .yumi_i(w_out_ready && w_out_valid),
    .v_o(w_out_valid),
    .data_o(w_out_data)
);

assign w_out_ready = out_ready_i;
assign out_valid_o = w_out_valid;
assign vector_data_o = w_out_data;

endmodule
