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

// ternip_pkg
//
// Ternip enums, the configuration struct, and helper functions.
//
// Import this package anywhere Ternip modules need the common enums or helpers.
// The ternip_assertions module at the bottom provides elaboration-time checks
// for supported configuration combinations.

package ternip_pkg;

// =========================== //
// Implementation-select enums //
// =========================== //
typedef enum logic [1:0] {
    MUL_BSG,
    MUL_ROUNDROBIN,
    MUL_STAR,
    MUL_NONE
} mul_impl_e;

// Piecewise-linear sigmoid approximations, symmetric about (0, 1/2). Each is the
// minimax fit for its segment count; the POWER2 variants restrict every slope to a
// power of two so the multiply degrades to a shift. Max |error| vs true sigmoid:
//   LUT                                 exact (2**FixedPointPrecision entries)
//   APPROXIMATE_1ST_ORDER               0.056050
//   APPROXIMATE_3RD_ORDER               0.017376
//   APPROXIMATE_5TH_ORDER               0.008362
//   APPROXIMATE_POWER2_SLOPE_1ST_ORDER  0.119203  (the long-standing hard sigmoid)
//   APPROXIMATE_POWER2_SLOPE_3RD_ORDER  0.034857
//   APPROXIMATE_POWER2_SLOPE_5TH_ORDER  0.015848
typedef enum logic [2:0] {
    SIGMOID_LUT,
    SIGMOID_APPROXIMATE_1ST_ORDER,
    SIGMOID_APPROXIMATE_3RD_ORDER,
    SIGMOID_APPROXIMATE_5TH_ORDER,
    SIGMOID_APPROXIMATE_POWER2_SLOPE_1ST_ORDER,
    SIGMOID_APPROXIMATE_POWER2_SLOPE_3RD_ORDER,
    SIGMOID_APPROXIMATE_POWER2_SLOPE_5TH_ORDER
} sigmoid_model_e;

function automatic int sigmoid_segment_count(sigmoid_model_e model);
    case (model)
        SIGMOID_APPROXIMATE_1ST_ORDER,
        SIGMOID_APPROXIMATE_POWER2_SLOPE_1ST_ORDER: return 1;
        SIGMOID_APPROXIMATE_3RD_ORDER,
        SIGMOID_APPROXIMATE_POWER2_SLOPE_3RD_ORDER: return 3;
        SIGMOID_APPROXIMATE_5TH_ORDER,
        SIGMOID_APPROXIMATE_POWER2_SLOPE_5TH_ORDER: return 5;
        default:                                    return 0;   // LUT
    endcase
endfunction

// Upper bound of segment `index`; below segment 0's lower bound the output is 0,
// at or above the last bound it is 1.
function automatic bit sigmoid_slopes_are_powers_of_two(sigmoid_model_e model);
    case (model)
        SIGMOID_APPROXIMATE_POWER2_SLOPE_1ST_ORDER,
        SIGMOID_APPROXIMATE_POWER2_SLOPE_3RD_ORDER,
        SIGMOID_APPROXIMATE_POWER2_SLOPE_5TH_ORDER: return 1;
        default:                                    return 0;
    endcase
endfunction

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

typedef enum logic [1:0] {
    DIV_BSG,
    DIV_ROUNDROBIN,
    DIV_NONE
} div_impl_e;

// ================= //
// Ternip parameters //
// ================= //
typedef struct packed {
    int unsigned D;
    int unsigned TmatmulParallelism;
    int unsigned VectorParallelism;
    int unsigned LutParallelism;
    int unsigned FixedPointPrecision;
    int          FixedPointExponent;
    sigmoid_model_e SigmoidModel;
    int unsigned BatchSize;
    int unsigned NumVectorRegisters;
    int unsigned ImmediateWidth;
    int unsigned DdrAddressWidth;
    int unsigned InstructionWidth;
    int unsigned DdrDataWidth;
    int unsigned AxiAuxDataWidth;
    int unsigned InstrFetchWidth;
    int unsigned CoreInterconnectNumStages;
    mul_impl_e   MultiplicationImplementation;
    div_impl_e   DivisionImplementation;
} ternip_cfg_t;

// ================================================================================= //
// Instruction fields and widths                                                     //
// The instruction_t type is derived from the user base struct in ternip_types#(Cfg) //
// ================================================================================= //
typedef enum logic [3:0] {
    NOP,
    ADD,
    SUB,
    MUL,
    DIV,
    SIG,
    CSIG,
    SILU
} rowwise_op_e;

typedef enum logic [1:0] {
    NO_LS_OP,
    LDV,
    SV
} loadstore_op_e;

typedef enum logic [1:0] {
    NO_TMATMUL_OP,
    IMPORT,
    GO,
    EXPORT
} tmatmul_op_e;

typedef enum logic [2:0] {
    NO_RMS_OP,
    CLEAR,
    ACCUMULATE,
    FINISH_ACCUMULATE,
    NORM
} rms_op_e;

typedef enum logic [2:0] {
    NO_FU,
    LOADSTORE,
    ROWWISE_OPERATION,
    TMATMUL,
    RMS,
    STALL
} fu_e;

// ======================================================= //
// Helper functions for fixed-point and integer operations //
// ======================================================= //
function automatic integer abs_int(integer a);
    return ((a<0) ? -a : a);
endfunction

function automatic integer max_int(integer a, integer b);
    return ((a>b) ? a : b);
endfunction

function automatic integer min_int(integer a, integer b);
    return ((a<b) ? a : b);
endfunction

function automatic integer clamp_int(integer lo, integer x, integer hi);
    return max_int(lo, min_int(x, hi));
endfunction

function automatic integer fixed_point_min(integer precision);
    if ((precision < 1) || (precision > $bits(integer)))
        $fatal(1, "fixed_point_min: precision %0d outside representable range [1, %0d]", precision, $bits(integer));
    return (1 << (precision-1));
endfunction

function automatic integer fixed_point_max(integer precision);
    if ((precision < 1) || (precision > $bits(integer)))
        $fatal(1, "fixed_point_max: precision %0d outside representable range [1, %0d]", precision, $bits(integer));
    return (1 << (precision-1)) - 1;
endfunction

function automatic integer fixed_point_one(integer exponent);
    if ((-exponent < 0) || (-exponent > $bits(integer)-1))
        $fatal(1, "fixed_point_one: exponent %0d yields a shift outside representable range [0, %0d]", exponent, $bits(integer)-1);
    return (1 <<< -exponent);
endfunction

// ==================================================== //
// 2-bit ternary type for ternary matrix multiplication //
// ==================================================== //
typedef logic signed [1:0] ternary_t;


endpackage : ternip_pkg

/* verilator lint_save */
/* verilator lint_off DECLFILENAME */
module ternip_assertions; import ternip_pkg::*;

// The instruction_t width check lives in ternip_types_assertions: instruction_t
// now lives only in ternip_types#(Cfg), which is declared after this package.
localparam ternip_cfg_t Cfg = `TERNIP_CFG;
if (!(Cfg.FixedPointPrecision inside {8, 16})) $fatal(0, "Invalid value for FixedPointPrecision: %0d.", Cfg.FixedPointPrecision);

endmodule
/* verilator lint_restore */
