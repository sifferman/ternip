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

    // y = slope*x + intercept on whichever segment holds x; 0 below the first
    // segment, 1 at or above the last bound.
    //
    // The segment is chosen FIRST and only then is the arithmetic done, so this
    // costs one shift/multiply rather than one per segment. Evaluating every
    // segment in parallel and muxing the results afterwards cost ~2.8 ns of
    // slack on a D=2048 build -- the model was fine, the structure was not.
    //
    // POWER2 models have every slope a negative power of two, so the scaling is
    // a variable right shift. The bias term makes that shift truncate toward
    // zero rather than floor, which is what reproduces the long-standing hard
    // sigmoid bit-for-bit under SIGMOID_APPROXIMATE_POWER2_SLOPE_1ST_ORDER.
    localparam int NumSegments = ternip_pkg::sigmoid_segment_count(SigmoidModel);
    localparam bit SlopesArePowersOfTwo = ternip_pkg::sigmoid_slopes_are_powers_of_two(SigmoidModel);
    localparam int SlopeFractionBits = 16;

    fixed_point_t selected_intercept;
    logic [7:0]   selected_shift;      // POWER2 path: log2(1/slope)
    longint       selected_slope;      // general path: slope << SlopeFractionBits
    logic         below_first_segment;
    logic         above_last_segment;

    always_comb begin
        selected_intercept  = '0;
        selected_shift      = '0;
        selected_slope      = 0;
        below_first_segment = 0;
        above_last_segment  = 1;
        for (int segment_index = NumSegments-1; segment_index >= 0; segment_index--) begin
            if (a_i < fixed_point_t'($rtoi(
                    ternip_pkg::sigmoid_segment_upper_bound(SigmoidModel, segment_index)
                    * (2.0 ** (-FixedPointExponent))
                    + ((ternip_pkg::sigmoid_segment_upper_bound(SigmoidModel, segment_index) < 0.0)
                       ? -0.5 : 0.5)))) begin
                selected_intercept = fixed_point_t'($rtoi(
                    ternip_pkg::sigmoid_segment_intercept(SigmoidModel, segment_index)
                    * (2.0 ** (-FixedPointExponent)) + 0.5));
                // Plain assignment, not a width cast: sv2v leaves a literal-width
                // cast like 8'(...) untranslated and Vivado's Verilog parser
                // rejects it. Assignment to the 8-bit target truncates the same way.
                selected_shift = $rtoi(
                    -$ln(ternip_pkg::sigmoid_segment_slope(SigmoidModel, segment_index)) / $ln(2.0) + 0.5);
                selected_slope = longint'(
                    ternip_pkg::sigmoid_segment_slope(SigmoidModel, segment_index)
                    * (2.0 ** SlopeFractionBits) + 0.5);
                above_last_segment = 0;
            end
        end
        below_first_segment = (a_i < fixed_point_t'($rtoi(
            -ternip_pkg::sigmoid_segment_upper_bound(SigmoidModel, NumSegments-1)
            * (2.0 ** (-FixedPointExponent)) - 0.5)));
    end

    fixed_point_t scaled_input;

    if (SlopesArePowersOfTwo) begin : gen_shift_scale
        // Bias makes the arithmetic shift truncate toward zero, matching a divide.
        fixed_point_t truncation_bias;
        assign truncation_bias = (a_i < 0)
                               ? fixed_point_t'((1 << selected_shift) - 1)
                               : '0;
        assign scaled_input = fixed_point_t'((a_i + truncation_bias) >>> selected_shift);
    end else begin : gen_multiply_scale
        logic signed [FixedPointPrecision+SlopeFractionBits+1:0] product;
        assign product = selected_slope * a_i;
        assign scaled_input = fixed_point_t'(product / (2 ** SlopeFractionBits));
    end

    always_comb begin
        if (below_first_segment)     y_o = '0;
        else if (above_last_segment) y_o = FixedPointOne;
        else                         y_o = scaled_input + selected_intercept;
    end

end

endmodule
