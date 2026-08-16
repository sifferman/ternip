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

`include "ternip_readmem_path.svh"

// ternip_silu
//
// Scalar fixed-point SiLU.
//
// Computes y_o = a_i * sigmoid(a_i). With UseHardSigmoid set, this uses a
// first-order approximation. Otherwise it reads a precomputed LUT.

module ternip_silu #(
    parameter ternip_pkg::ternip_cfg_t Cfg = `TERNIP_CFG,

    parameter int FixedPointPrecision = Cfg.FixedPointPrecision,
    parameter int FixedPointExponent  = Cfg.FixedPointExponent,
    parameter bit UseHardSigmoid      = Cfg.UseHardSigmoid,
    parameter ternip_pkg::mul_impl_e MultiplicationImplementation = Cfg.MultiplicationImplementation,

    localparam type fixed_point_t = logic signed [FixedPointPrecision-1:0]
) (
    input  logic         clk_i,
    input  logic         rst_ni,

    output logic         in_ready_o,
    input  logic         in_valid_i,
    input  fixed_point_t a_i,

    input  logic         out_ready_i,
    output logic         out_valid_o,
    output fixed_point_t y_o
);

localparam fixed_point_t FixedPointOne = ternip_pkg::fixed_point_one(FixedPointExponent);
localparam int FixedPointUnaryOperationLutSize = UseHardSigmoid ? 1 : (2 ** FixedPointPrecision);

if (UseHardSigmoid) begin : gen_hard_silu

    // hard_silu(x) = x * clamp(x/4 + 0.5, 0, 1)
    fixed_point_t sig_result;

    always_comb begin
        fixed_point_t linear;
        linear = (a_i / 4) + (FixedPointOne / 2);
        if (linear <= 0)
            sig_result = '0;
        else if (linear >= FixedPointOne)
            sig_result = FixedPointOne;
        else
            sig_result = linear;
    end

    // Pipeline register — breaks combinational path from sigmoid into ternip_mul.
    // in_ready_o: accept when stage is empty or ternip_mul is consuming this cycle.
    logic         stage_valid_d, stage_valid_q;
    fixed_point_t stage_a_d,     stage_a_q;
    fixed_point_t stage_sig_d,   stage_sig_q;

    logic mul_in_ready;

    assign in_ready_o = !stage_valid_q || mul_in_ready;

    always_comb begin
        stage_valid_d = stage_valid_q;
        stage_a_d     = stage_a_q;
        stage_sig_d   = stage_sig_q;
        if (in_ready_o) begin
            stage_valid_d = in_valid_i;
            stage_a_d     = a_i;
            stage_sig_d   = sig_result;
        end
    end

    always_ff @(posedge clk_i) begin
        if (!rst_ni)
            stage_valid_q <= 1'b0;
        else
            stage_valid_q <= stage_valid_d;
    end
    always_ff @(posedge clk_i) begin
        stage_a_q   <= stage_a_d;
        stage_sig_q <= stage_sig_d;
        `ifndef SYNTHESIS
        if (!rst_ni) begin
            stage_a_q   <= 'x;
            stage_sig_q <= 'x;
        end
        `endif
    end

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

        .in_ready_o(mul_in_ready),
        .in_valid_i(stage_valid_q),
        .a_i(stage_a_q),
        .b_i(stage_sig_q),

        .out_ready_i,
        .out_valid_o,
        .y_o
    );

end else begin : gen_lut_silu

    fixed_point_t SILU_LUT [FixedPointUnaryOperationLutSize];
    initial $readmemh(`READMEM_PATH(LUT_silu_FixedPoint_to_FixedPoint.memh), SILU_LUT);
    assign y_o = SILU_LUT[$unsigned(a_i)];
    assign in_ready_o = out_ready_i;
    assign out_valid_o = in_valid_i;

end

endmodule
