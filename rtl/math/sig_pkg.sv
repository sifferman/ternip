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

// sig_pkg
//
// The piecewise-linear sigmoid segment tables and the arithmetic that turns them
// into the constants ternip_sig holds.

package sig_pkg;

import ternip_pkg::*;

function automatic int sigmoid_segment_count(sigmoid_model_e model);
    case (model)
        SIGMOID_APPROXIMATE_1ST_ORDER,
        SIGMOID_APPROXIMATE_POWER2_SLOPE_1ST_ORDER: return 1;
        SIGMOID_APPROXIMATE_3RD_ORDER,
        SIGMOID_APPROXIMATE_POWER2_SLOPE_3RD_ORDER: return 3;
        SIGMOID_APPROXIMATE_5TH_ORDER,
        SIGMOID_APPROXIMATE_POWER2_SLOPE_5TH_ORDER: return 5;
        default:                                    return 0; // LUT
    endcase
endfunction

// One segment, in the integer arithmetic the hardware performs: the multiplying
// models scale by scaled_slope, the POWER2 models shift right by right_shift_amount.
typedef struct packed {
    longint upper_bound;
    longint intercept;
    longint scaled_slope;
    int     right_shift_amount;
} sigmoid_segment_t;

// Upper bound of segment `index`; below segment 0's lower bound the
// output is 0, at or above the last bound it is 1.
function automatic real sigmoid_segment_upper_bound(sigmoid_model_e model, int index);
    case (model)
        SIGMOID_APPROXIMATE_1ST_ORDER: return 2.823822;
        SIGMOID_APPROXIMATE_3RD_ORDER:
            case (index) 0: return -1.652934; 1: return 1.652934; default: return 4.035162; endcase
        SIGMOID_APPROXIMATE_5TH_ORDER:
            case (index) 0: return -2.508140; 1: return -1.243333; 2: return 1.243333;
                         3: return 2.508140; default: return 4.775714; endcase
        SIGMOID_APPROXIMATE_POWER2_SLOPE_1ST_ORDER: return 2.0;
        SIGMOID_APPROXIMATE_POWER2_SLOPE_3RD_ORDER:
            case (index) 0: return -1.245525; 1: return 1.245525; default: return 4.263425; endcase
        SIGMOID_APPROXIMATE_POWER2_SLOPE_5TH_ORDER:
            case (index) 0: return -2.559516; 1: return -0.938899; 2: return 0.938899;
                         3: return 2.559516; default: return 4.565853; endcase
        default: return 0.0;
    endcase
endfunction

function automatic real sigmoid_segment_slope(sigmoid_model_e model, int index);
    case (model)
        SIGMOID_APPROXIMATE_1ST_ORDER: return 0.177065;
        SIGMOID_APPROXIMATE_3RD_ORDER:
            case (index) 1: return 0.215776; default: return 0.060169; endcase
        SIGMOID_APPROXIMATE_5TH_ORDER:
            case (index) 2: return 0.228825; 1, 3: return 0.117462; default: return 0.029515; endcase
        SIGMOID_APPROXIMATE_POWER2_SLOPE_1ST_ORDER: return 0.25;
        SIGMOID_APPROXIMATE_POWER2_SLOPE_3RD_ORDER:
            case (index) 1: return 0.25; default: return 0.0625; endcase
        SIGMOID_APPROXIMATE_POWER2_SLOPE_5TH_ORDER:
            case (index) 2: return 0.25; 1, 3: return 0.125; default: return 0.03125; endcase
        default: return 0.0;
    endcase
endfunction

function automatic real sigmoid_segment_intercept(sigmoid_model_e model, int index);
    case (model)
        SIGMOID_APPROXIMATE_1ST_ORDER: return 0.5;
        SIGMOID_APPROXIMATE_3RD_ORDER:
            case (index) 0: return 0.242793; 1: return 0.5; default: return 0.757207; endcase
        SIGMOID_APPROXIMATE_5TH_ORDER:
            case (index) 0: return 0.140956; 1: return 0.361539; 2: return 0.5;
                         3: return 0.638461; default: return 0.859044; endcase
        SIGMOID_APPROXIMATE_POWER2_SLOPE_1ST_ORDER: return 0.5;
        SIGMOID_APPROXIMATE_POWER2_SLOPE_3RD_ORDER:
            case (index) 0: return 0.266464; 1: return 0.5; default: return 0.733536; endcase
        SIGMOID_APPROXIMATE_POWER2_SLOPE_5TH_ORDER:
            case (index) 0: return 0.142683; 1: return 0.382638; 2: return 0.5;
                         3: return 0.617362; default: return 0.857317; endcase
        default: return 0.0;
    endcase
endfunction

function automatic int sigmoid_segment_right_shift_amount(sigmoid_model_e model, int index);
    for (int right_shift_amount = 0; right_shift_amount < $bits(integer); right_shift_amount++)
        if ((sigmoid_segment_slope(model, index) * (2.0 ** right_shift_amount)) >= 1.0) return right_shift_amount;
    return 0;
endfunction

function automatic bit sigmoid_slopes_are_powers_of_two(sigmoid_model_e model);
    return model inside {SIGMOID_APPROXIMATE_POWER2_SLOPE_1ST_ORDER,
                         SIGMOID_APPROXIMATE_POWER2_SLOPE_3RD_ORDER,
                         SIGMOID_APPROXIMATE_POWER2_SLOPE_5TH_ORDER};
endfunction

function automatic sigmoid_segment_t sigmoid_segment(sigmoid_model_e model, int index,
                                                     integer fixed_point_exponent, integer slope_fraction_bits);
    return '{
        upper_bound:  real2fixed_point(sigmoid_segment_upper_bound(model, index), fixed_point_exponent, $bits(longint)),
        intercept:    real2fixed_point(sigmoid_segment_intercept(model, index), fixed_point_exponent, $bits(longint)),
        scaled_slope: real2fixed_point(sigmoid_segment_slope(model, index), -slope_fraction_bits, $bits(longint)),
        right_shift_amount: sigmoid_segment_right_shift_amount(model, index)
    };
endfunction

localparam int MaxSigmoidSegments = sigmoid_segment_count(SIGMOID_APPROXIMATE_5TH_ORDER);

// The whole table, so a module holds it as one localparam and no real reaches its body.
function automatic sigmoid_segment_t [MaxSigmoidSegments-1:0] sigmoid_segments(
        sigmoid_model_e model, integer fixed_point_exponent, integer slope_fraction_bits);
    sigmoid_segments = '0;
    for (int index = 0; index < sigmoid_segment_count(model); index++)
        sigmoid_segments[index] =
            sigmoid_segment(model, index, fixed_point_exponent, slope_fraction_bits);
endfunction

endpackage : sig_pkg
