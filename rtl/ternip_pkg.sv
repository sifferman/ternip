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

function automatic real clamp_real(real lo, real x, real hi);
    return (x < lo) ? lo : ((x > hi) ? hi : x);
endfunction

function automatic longint fixed_point_min(integer precision);
    if ((precision < 1) || (precision > $bits(longint)))
        $fatal(1, "fixed_point_min: precision %0d outside representable range [1, %0d]", precision, $bits(longint));
    return -(64'sd1 <<< (precision-1));
endfunction

function automatic longint fixed_point_max(integer precision);
    if ((precision < 1) || (precision > $bits(longint)))
        $fatal(1, "fixed_point_max: precision %0d outside representable range [1, %0d]", precision, $bits(longint));
    return (64'sd1 <<< (precision-1)) - 1;
endfunction

function automatic integer fixed_point_one(integer exponent);
    if ((-exponent < 0) || (-exponent > $bits(integer)-1))
        $fatal(1, "fixed_point_one: exponent %0d yields a shift outside representable range [0, %0d]", exponent, $bits(integer)-1);
    return (1 <<< -exponent);
endfunction

function automatic real fixed_point2real(longint value, integer exponent);
    return value * (2.0 ** exponent);
endfunction

function automatic longint real2fixed_point(real value, integer exponent, integer precision);
    real lowest  = fixed_point_min(precision) * 1.0;
    real highest = fixed_point_max(precision) * 1.0;
    return longint'(clamp_real(lowest, value * (2.0 ** -exponent), highest));
endfunction

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
// minimax fit for its segment count; the POWER2_SLOPE variants restrict every slope to a
// power of two so the multiply degrades to a shift.
typedef enum logic [2:0] {
    SIGMOID_LUT,                                // Max |error|: exact
    SIGMOID_APPROXIMATE_1ST_ORDER,              // Max |error|: 0.056050
    SIGMOID_APPROXIMATE_3RD_ORDER,              // Max |error|: 0.017376
    SIGMOID_APPROXIMATE_5TH_ORDER,              // Max |error|: 0.008362
    SIGMOID_APPROXIMATE_POWER2_SLOPE_1ST_ORDER, // Max |error|: 0.119203  ("hard sigmoid")
    SIGMOID_APPROXIMATE_POWER2_SLOPE_3RD_ORDER, // Max |error|: 0.034857
    SIGMOID_APPROXIMATE_POWER2_SLOPE_5TH_ORDER  // Max |error|: 0.015848
} sigmoid_model_e;

typedef enum logic [1:0] {
    DIV_BSG,
    DIV_ROUNDROBIN,
    DIV_NONE
} div_impl_e;

// ================= //
// Ternip parameters //
// ================= //
typedef struct packed {
    int unsigned    D;
    int unsigned    TmatmulParallelism;
    int unsigned    VectorParallelism;
    int unsigned    LutParallelism;
    int unsigned    FixedPointPrecision;
    int             FixedPointExponent;
    sigmoid_model_e SigmoidModel;
    int unsigned    BatchSize;
    int unsigned    NumVectorRegisters;
    int unsigned    ImmediateWidth;
    int unsigned    DdrAddressWidth;
    int unsigned    InstructionWidth;
    int unsigned    DdrDataWidth;
    int unsigned    AxiAuxDataWidth;
    int unsigned    InstrFetchWidth;
    int unsigned    CoreInterconnectNumStages;
    mul_impl_e      MultiplicationImplementation;
    div_impl_e      DivisionImplementation;
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
