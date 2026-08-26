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

// ternip_types
//
// Config-derived VALUE + TYPE namespace, parameterized by the user base
// configuration struct.
//   - ternip_pkg holds the user base struct (ternip_cfg_t) and the enums.
//   - ternip_types#(Cfg) derives every config-DEPENDENT integer value AND data
//     type from those base parameters (Cfg is a ternip_cfg_t).

class ternip_types #(
    parameter ternip_pkg::ternip_cfg_t Cfg = `TERNIP_CFG
);

    localparam int FixedPointUnaryOperationLutSize = (Cfg.SigmoidModel == ternip_pkg::SIGMOID_LUT) ? (2 ** Cfg.FixedPointPrecision) : 1;

    localparam int VectorSizeInBytes     = Cfg.D * Cfg.FixedPointPrecision / 8;
    localparam int BytesPerFixedPointNum = Cfg.FixedPointPrecision / 8;
    localparam int MatrixSizeInBytes     = Cfg.D * Cfg.D * 2 / 8;
    localparam int TernaryWeightsPerByte = 8 / 2;

    localparam int RmsSqaSumPrecision          = 2 * Cfg.FixedPointPrecision;
    localparam int RmsSqaSumExponent           = 2 * Cfg.FixedPointExponent;
    localparam int RmsValueReciprocalPrecision = 2 * (Cfg.FixedPointPrecision + 1);
    localparam int RmsValueReciprocalExponent  = -int'(Cfg.FixedPointPrecision);
    localparam int RmsSqrtInputPrecision       = 2 * RmsValueReciprocalPrecision;
    localparam int RmsSqrtInputExponent        = 2 * RmsValueReciprocalExponent;

    localparam int RmsAccumulatorWidth = 2 * Cfg.FixedPointPrecision + $clog2(Cfg.D) + 1;

    localparam int RmsValueReciprocalUnaryOperationLutSize = (2 ** RmsValueReciprocalPrecision);
    localparam int RmsSqrtInputUnaryOperationLutSize       = (2 ** RmsSqrtInputPrecision);

    localparam int NumChunksPerVector = Cfg.D / Cfg.VectorParallelism;


    typedef logic signed [Cfg.FixedPointPrecision-1:0] fixed_point_t;
    localparam fixed_point_t FixedPointMin = ternip_pkg::fixed_point_min(Cfg.FixedPointPrecision);
    localparam fixed_point_t FixedPointMax = ternip_pkg::fixed_point_max(Cfg.FixedPointPrecision);
    localparam fixed_point_t FixedPointOne = ternip_pkg::fixed_point_one(Cfg.FixedPointExponent);


    typedef logic signed [RmsAccumulatorWidth-1:0] rms_accumulator_t;

    typedef logic signed [RmsSqaSumPrecision-1:0] rms_sqa_sum_t;
    localparam rms_sqa_sum_t RmsSqaSumMax = (1 << (RmsSqaSumPrecision-1)) - 1;
    localparam rms_sqa_sum_t RmsSqaSumMin = -(1 << (RmsSqaSumPrecision-1));

    typedef logic signed [RmsValueReciprocalPrecision-1:0] rms_value_reciprocal_t;
    localparam rms_value_reciprocal_t RmsValueReciprocalMax = (1 << (RmsValueReciprocalPrecision-1)) - 1;
    localparam rms_value_reciprocal_t RmsValueReciprocalMin = -(1 << (RmsValueReciprocalPrecision-1));

    typedef logic signed [RmsSqrtInputPrecision-1:0] rms_sqrt_input_t;
    localparam rms_sqrt_input_t RmsSqrtInputMax = (1 << (RmsSqrtInputPrecision-1)) - 1;
    localparam rms_sqrt_input_t RmsSqrtInputMin = -(1 << (RmsSqrtInputPrecision-1));


    typedef fixed_point_t [Cfg.VectorParallelism-1:0] vector_chunk_t;
    typedef logic [$clog2(NumChunksPerVector)-1:0] vector_offset_t;
    typedef logic [$clog2(Cfg.NumVectorRegisters)-1:0] vector_select_t;
    typedef logic [Cfg.DdrAddressWidth-1:0] ddr_address_t;
    typedef logic [Cfg.ImmediateWidth-1:0] immediate_t;
    typedef ternip_pkg::ternary_t [Cfg.TmatmulParallelism-1:0] tmatmul_stream_data_t;


    // Packed instruction layout (config-independent enums come from ternip_pkg)
    localparam int _InstructionUnusedBitsWidth =
        Cfg.InstructionWidth
        - ($bits(ternip_pkg::fu_e)
           + $bits(ternip_pkg::rowwise_op_e)
           + $clog2(Cfg.NumVectorRegisters) * 3
           + $bits(ternip_pkg::loadstore_op_e)
           + $bits(ternip_pkg::tmatmul_op_e)
           + $bits(ternip_pkg::rms_op_e)
           + Cfg.DdrAddressWidth);

    typedef struct packed {
        logic [_InstructionUnusedBitsWidth-1:0] _unused;
        ternip_pkg::fu_e fu;
        ternip_pkg::rowwise_op_e rowwise_op;
        vector_select_t v_a;
        vector_select_t v_b;
        vector_select_t v_y;
        ternip_pkg::loadstore_op_e loadstore_op;
        ternip_pkg::tmatmul_op_e tmatmul_op;
        ternip_pkg::rms_op_e rms_op;
        ddr_address_t ddr_address;
    } instruction_t;

endclass

/* verilator lint_save */
/* verilator lint_off DECLFILENAME */
module ternip_types_assertions; import ternip_pkg::*;

// instruction_t width check (instruction_t lives only in ternip_types#(Cfg)).
localparam ternip_cfg_t Cfg = `TERNIP_CFG;
if ($bits(ternip_types#(Cfg)::instruction_t) != Cfg.InstructionWidth)
    $fatal(0, "Expected an instruction_t width of %0d, but received %0d.", Cfg.InstructionWidth, $bits(ternip_types#(Cfg)::instruction_t));

endmodule
/* verilator lint_restore */
