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

// ternip_sig
//
// Scalar fixed-point sigmoid.
//
// Computes y_o = sigmoid(a_i). SigmoidModel selects a precomputed LUT (exact,
// but 2**FixedPointPrecision entries) or one of six piecewise-linear minimax
// approximations; see sigmoid_model_e for their errors. This is combinational.

module ternip_sig #(
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

if (SigmoidModel == ternip_pkg::SIGMOID_LUT) begin : gen_lut_sig

    fixed_point_t SIGMOID_LUT [FixedPointUnaryOperationLutSize];
    initial $readmemh(`READMEM_PATH(LUT_sig_FixedPoint_to_FixedPoint.memh), SIGMOID_LUT);
    assign y_o = SIGMOID_LUT[$unsigned(a_i)];

end else begin : gen_piecewise_sig

    // The segment is chosen FIRST and only then is the arithmetic done, so this
    // costs one shift/multiply rather than one per segment.
    localparam int NumSegments          = sig_pkg::sigmoid_segment_count(SigmoidModel);
    localparam bit SlopesArePowersOfTwo = sig_pkg::sigmoid_slopes_are_powers_of_two(SigmoidModel);
    localparam int SlopeFractionBits    = 16;

    localparam sig_pkg::sigmoid_segment_t [sig_pkg::MaxSigmoidSegments-1:0] Segments =
        sig_pkg::sigmoid_segments(SigmoidModel, FixedPointExponent, SlopeFractionBits);
    localparam fixed_point_t LastSegmentUpperBound  = Segments[NumSegments-1].upper_bound;
    localparam fixed_point_t FirstSegmentLowerBound = -LastSegmentUpperBound;

    sig_pkg::sigmoid_segment_t selected_segment;
    logic                         input_above_every_segment;

    always_comb begin
        selected_segment          = '0;
        input_above_every_segment = 1;
        for (int segment_index = NumSegments-1; segment_index >= 0; segment_index--)
            if (a_i < Segments[segment_index].upper_bound) begin
                selected_segment          = Segments[segment_index];
                input_above_every_segment = 0;
            end
    end

    fixed_point_t scaled_input;

    if (SlopesArePowersOfTwo) begin : gen_right_shift_by_slope
        fixed_point_t truncation_bias;
        assign truncation_bias = (a_i < 0)
                               ? fixed_point_t'((1 << selected_segment.right_shift_amount) - 1)
                               : '0;
        assign scaled_input = fixed_point_t'((a_i + truncation_bias) >>> selected_segment.right_shift_amount);
    end else begin : gen_multiply_by_slope
        logic signed [FixedPointPrecision+SlopeFractionBits+1:0] product;
        assign product = selected_segment.scaled_slope * a_i;
        assign scaled_input = fixed_point_t'(product / (2 ** SlopeFractionBits));
    end

    assign y_o = (a_i < FirstSegmentLowerBound) ? '0
               : input_above_every_segment      ? FixedPointOne
               : scaled_input + fixed_point_t'(selected_segment.intercept);

end

endmodule
