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

// ternip_csig
//
// Scalar fixed-point complementary sigmoid.
//
// Computes y_o = 1 - sigmoid(a_i). SIGMOID_LUT reads a
// precomputed LUT; every approximation instantiates ternip_sig and subtracts,
// which is exact for the complement and avoids duplicating the approximation.
// This is combinational.

module ternip_csig #(
    parameter ternip_pkg::ternip_cfg_t Cfg = `TERNIP_CFG,

    parameter int  FixedPointPrecision = Cfg.FixedPointPrecision,
    parameter int  FixedPointExponent  = Cfg.FixedPointExponent,
    parameter ternip_pkg::sigmoid_model_e SigmoidModel = Cfg.SigmoidModel,

    localparam type fixed_point_t = logic signed [FixedPointPrecision-1:0]
) (
    input  fixed_point_t a_i,
    output fixed_point_t y_o
);

localparam fixed_point_t FixedPointOne = ternip_pkg::fixed_point_one(FixedPointExponent);
localparam int FixedPointUnaryOperationLutSize =
    (SigmoidModel == ternip_pkg::SIGMOID_LUT) ? (2 ** FixedPointPrecision) : 1;

if (SigmoidModel == ternip_pkg::SIGMOID_LUT) begin : gen_lut_csig

    fixed_point_t CSIGMOID_LUT [FixedPointUnaryOperationLutSize];
    initial $readmemh(`READMEM_PATH(LUT_csig_FixedPoint_to_FixedPoint.memh), CSIGMOID_LUT);
    assign y_o = CSIGMOID_LUT[$unsigned(a_i)];

end else begin : gen_approx_csig

    fixed_point_t sigmoid_result;

    ternip_sig #(
        .FixedPointPrecision(FixedPointPrecision),
        .FixedPointExponent(FixedPointExponent),
        .SigmoidModel(SigmoidModel)
    ) sigmoid (
        .a_i,
        .y_o(sigmoid_result)
    );

    assign y_o = FixedPointOne - sigmoid_result;

end

endmodule
