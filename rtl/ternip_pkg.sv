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
// Shared Ternip types, configuration-derived constants, and helper functions.
//
// This package defines instruction enums, fixed-point and vector data types,
// DDR address types, RMS internal formats, and the packed instruction_t layout.
// It includes the build-selected `CONFIG_FILENAME, so parameters such as D,
// FixedPointPrecision, VectorParallelism, and implementation choices come from
// the active configuration file.
//
// Import this package anywhere Ternip modules need common enums, widths, or
// fixed-point helpers. The ternip_assertions module at the bottom provides
// elaboration-time checks for supported configuration combinations.

package ternip_pkg;

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
    MUL_BSG,
    MUL_ROUNDROBIN,
    MUL_STAR,
    MUL_NONE
} mul_impl_e;

typedef enum logic [1:0] {
    DIV_BSG,
    DIV_ROUNDROBIN,
    DIV_NONE
} div_impl_e;

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
    return (1 << (precision-1));
endfunction

function automatic integer fixed_point_max(integer precision);
    return (1 << (precision-1)) - 1;
endfunction

function automatic integer fixed_point_one(integer exponent);
    return (1 <<< -exponent);
endfunction


`include `CONFIG_FILENAME


// Data Types

typedef logic signed [1:0] ternary_t;

typedef logic signed [FixedPointPrecision-1:0] fixed_point_t;
localparam fixed_point_t FixedPointMin = (1 << (FixedPointPrecision-1));
localparam fixed_point_t FixedPointMax = (FixedPointMin - 1);
localparam fixed_point_t FixedPointOne = (1 <<< -FixedPointExponent);
localparam int FixedPointUnaryOperationLutSize = UseHardSigmoid ? 1 : (2 ** FixedPointPrecision);

localparam int VectorSizeInBytes = D * $bits(fixed_point_t)/8;
localparam int BytesPerFixedPointNum = $bits(fixed_point_t)/8;
localparam int MatrixSizeInBytes = D*D * $bits(ternary_t)/8;
localparam int TernaryWeightsPerByte = 8 / $bits(ternary_t);


// RMS

localparam int RmsSqaSumPrecision = 2*FixedPointPrecision;
localparam int RmsSqaSumExponent = 2*FixedPointExponent;
localparam int RmsValueReciprocalPrecision = 2*(FixedPointPrecision + 1);
localparam int RmsValueReciprocalExponent = -FixedPointPrecision;
localparam int RmsSqrtInputPrecision = 2*RmsValueReciprocalPrecision;
localparam int RmsSqrtInputExponent = 2*RmsValueReciprocalExponent;

// RmsAccumulatorWidth is needed to store the sum of D RmsFixedPoint values without overflow
localparam int RmsAccumulatorWidth = RmsSqaSumPrecision + $clog2(D) + 1;
typedef logic signed [RmsAccumulatorWidth-1:0] rms_accumulator_t;

typedef logic signed [RmsSqaSumPrecision-1:0] rms_sqa_sum_t;
localparam rms_sqa_sum_t RmsSqaSumMax = (1 << (RmsSqaSumPrecision-1)) - 1;
localparam rms_sqa_sum_t RmsSqaSumMin = -(1 << (RmsSqaSumPrecision-1));

typedef logic signed [RmsValueReciprocalPrecision-1:0] rms_value_reciprocal_t;
localparam rms_value_reciprocal_t RmsValueReciprocalMax = (1 << (RmsValueReciprocalPrecision-1)) - 1;
localparam rms_value_reciprocal_t RmsValueReciprocalMin = -(1 << (RmsValueReciprocalPrecision-1));
localparam int RmsValueReciprocalUnaryOperationLutSize = (2 ** RmsValueReciprocalPrecision);

typedef logic signed [RmsSqrtInputPrecision-1:0] rms_sqrt_input_t;
localparam rms_sqrt_input_t RmsSqrtInputMax = (1 << (RmsSqrtInputPrecision-1)) - 1;
localparam rms_sqrt_input_t RmsSqrtInputMin = -(1 << (RmsSqrtInputPrecision-1));
localparam int RmsSqrtInputUnaryOperationLutSize = (2 ** RmsSqrtInputPrecision);


// Instruction Info

typedef fixed_point_t [VectorParallelism-1:0] vector_chunk_t;
localparam int NumChunksPerVector = D / VectorParallelism;

typedef logic [$clog2(NumChunksPerVector)-1:0] vector_offset_t;
typedef logic [$clog2(NumVectorRegisters)-1:0] vector_select_t;

typedef logic [DdrAddressWidth-1:0] ddr_address_t;
typedef logic [ImmediateWidth-1:0] immediate_t;

typedef ternary_t [TmatmulParallelism-1:0] tmatmul_stream_data_t;

// Address-carrying instructions (ldv, sv, tmatmul_go) stream their 64-bit
// addresses as trailing beats on the same instruction channel after this
// opcode beat. ldv/sv carry one address beat; tmatmul_go carries one beat per
// DDR bank (NumDdrBanksPerTmatmul total). The opcode beat itself only has to
// carry the rms_finish_accumulate length as an inline immediate.
localparam int InstructionUnusedBitsWidth =
    InstructionWidth
    - ($bits(fu_e)
       + $bits(rowwise_op_e)
       + $bits(vector_select_t)*3
       + $bits(loadstore_op_e)
       + $bits(tmatmul_op_e)
       + $bits(rms_op_e)
       + $bits(immediate_t));

typedef struct packed {
    logic [InstructionUnusedBitsWidth-1:0] _unused;
    fu_e fu;
    rowwise_op_e rowwise_op;
    vector_select_t v_a;
    vector_select_t v_b;
    vector_select_t v_y;
    loadstore_op_e loadstore_op;
    tmatmul_op_e tmatmul_op;
    rms_op_e rms_op;
    immediate_t immediate;
} instruction_t;

endpackage : ternip_pkg

/* verilator lint_save */
/* verilator lint_off DECLFILENAME */
module ternip_assertions; import ternip_pkg::*;

if ($bits(instruction_t)!=InstructionWidth) $fatal(0, "Expected an instruction_t width of %0d, but received %0d.", InstructionWidth, $bits(instruction_t));
if (!(FixedPointPrecision inside {8, 16})) $fatal(0, "Invalid value for FixedPointPrecision: %0d.", FixedPointPrecision);

// NumDdrBanksPerTmatmul = N: each ternip_tmatmul has N parallel (DDR stream +
// ternary_mul + MOA + gbfifo_export + exportvector) lanes feeding the same
// importvector. Row-wise contiguous matrix split: bank b owns rows
// [b*(D/N), (b+1)*(D/N)). All four divisibility conditions are required for
// the lane logic to be clean.
if (NumDdrBanksPerTmatmul < 1)
    $fatal(0, "NumDdrBanksPerTmatmul (%0d) must be >= 1.", NumDdrBanksPerTmatmul);
if (NumDdrBanksPerTmatmul > DramNumBanks)
    $fatal(0, "NumDdrBanksPerTmatmul (%0d) exceeds DramNumBanks (%0d).", NumDdrBanksPerTmatmul, DramNumBanks);
if (D % NumDdrBanksPerTmatmul != 0)
    $fatal(0, "D (%0d) must be divisible by NumDdrBanksPerTmatmul (%0d).", D, NumDdrBanksPerTmatmul);
if (MatrixSizeInBytes % NumDdrBanksPerTmatmul != 0)
    $fatal(0, "MatrixSizeInBytes (%0d) must be divisible by NumDdrBanksPerTmatmul (%0d).", MatrixSizeInBytes, NumDdrBanksPerTmatmul);
if ((D / NumDdrBanksPerTmatmul) % VectorParallelism != 0)
    $fatal(0, "Rows-per-bank (D/N = %0d) must be divisible by VectorParallelism (%0d) so each bank's exportvector packs cleanly into chunks.", D / NumDdrBanksPerTmatmul, VectorParallelism);

endmodule
/* verilator lint_restore */
