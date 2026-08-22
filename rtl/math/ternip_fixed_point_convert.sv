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

// ternip_fixed_point_convert
//
// Convert between signed fixed-point formats.
//
// Interprets in as InPrecision bits scaled by 2**InExponent. Produces out as
// OutPrecision bits scaled by 2**OutExponent, with roundToIntegralTiesToAway rounding.
//
// NumPipelineStages splits the same arithmetic across that many registered stages
// (latency only, value unchanged); the in/out ready-valid handshake backpressures them.
//   NumPipelineStages == 0 : combinational, clk_i/rst_ni unused
//   NumPipelineStages == 1 : register after the absolute-value negate
//   NumPipelineStages == 2 : additional register after the round add

module ternip_fixed_point_convert #(
    parameter int InPrecision          = 16,
    parameter int InExponent           = 0,
    parameter int OutPrecision         = 16,
    parameter int OutExponent          = 0,
    parameter int NumPipelineStages    = 0
) (
    input  logic                           clk_i,
    input  logic                           rst_ni,

    output logic                           in_ready_o,
    input  logic                           in_valid_i,
    input  logic signed [InPrecision-1:0]  in,

    input  logic                           out_ready_i,
    output logic                           out_valid_o,
    output logic signed [OutPrecision-1:0] out
);

    if (InPrecision < 1) $fatal(0, "InPrecision (%0d) must be positive", InPrecision);
    if (OutPrecision < 1) $fatal(0, "OutPrecision (%0d) must be positive", OutPrecision);
    if (NumPipelineStages < 0 || NumPipelineStages > 2)
        $fatal(0, "NumPipelineStages (%0d) must be 0, 1, or 2", NumPipelineStages);

    function automatic integer max_int(integer a, integer b);
        return ((a>b) ? a : b);
    endfunction

    typedef logic signed [OutPrecision-1:0] out_t;

    localparam int MaxPrecision = max_int(InPrecision, OutPrecision) + 2; // 2 extra bits for abs and rounding
    localparam int LeftShiftAmount = InExponent - OutExponent;
    localparam logic signed [OutPrecision-1:0] OutMin = -(1 <<< (OutPrecision-1));
    localparam logic signed [OutPrecision-1:0] OutMax =  (1 <<< (OutPrecision-1)) - 1;

    localparam logic [MaxPrecision-1:0] guard_bit_mask  = 1 << max_int(0, -LeftShiftAmount);
    localparam logic [MaxPrecision-1:0] round_bit_mask  = guard_bit_mask >> 1;
    localparam logic [MaxPrecision-1:0] sticky_bits_mask = (round_bit_mask != 0)
                                                         ? (round_bit_mask - 1)
                                                         : '0;

    localparam int NumNonClampingBits = max_int(0, OutPrecision-1 - LeftShiftAmount);
    localparam logic [MaxPrecision-1:0] clamping_bits_mask = (MaxPrecision'('1) << NumNonClampingBits);

    // Elastic handshake: advance the whole pipeline (accept a new input and shift
    // the stages together) whenever the output slot is empty or being consumed, so
    // backpressure flows through the in/out ready-valid instead of a bare enable.
    wire advance = out_ready_i || !out_valid_o;
    assign in_ready_o = advance;

    if (NumPipelineStages == 0) begin : g_valid_comb
        assign out_valid_o = in_valid_i;
    end else begin : g_valid_pipe
        logic [NumPipelineStages-1:0] valid_d, valid_q;
        assign valid_d = (valid_q << 1) | in_valid_i;
        always_ff @(posedge clk_i) begin
            if (!rst_ni) begin
                valid_q <= '0;
            end else if (advance) begin
                valid_q <= valid_d;
            end
        end
        assign out_valid_o = valid_q[NumPipelineStages-1];
    end

    // ---------------------------------------------------------------------
    // Stage 0 (always combinational, from `in`): sign + absolute value.
    // The _d1 nets feed the optional stage-1 register.
    // ---------------------------------------------------------------------
    wire                            in_sign_d1 = (in < 0);
    wire signed [MaxPrecision-1:0]  in_abs_d1  = in_sign_d1 ? -in : in;

    // Optional register after the absolute-value negate.
    logic signed [MaxPrecision-1:0] in_abs_q1;
    logic                           in_sign_q1;
    if (NumPipelineStages >= 1) begin : g_abs_reg
        always_ff @(posedge clk_i) begin
            if (advance) begin
                in_abs_q1  <= in_abs_d1;
                in_sign_q1 <= in_sign_d1;
            end
            `ifndef SYNTHESIS
            if (!rst_ni) begin
                in_abs_q1  <= 'x;
                in_sign_q1 <= 'x;
            end
            `endif
        end
    end else begin : g_abs_comb
        assign in_abs_q1  = in_abs_d1;
        assign in_sign_q1 = in_sign_d1;
    end

    // Value at stage 1 (registered when NumPipelineStages >= 1, else passthrough).
    wire signed [MaxPrecision-1:0]  in_abs_stage1  = in_abs_q1;
    wire                            in_sign_stage1 = in_sign_q1;

    // ---------------------------------------------------------------------
    // Stage 1 (combinational, from in_abs_stage1): rounding + clamp detection.
    // The _d2 nets feed the optional stage-2 register.
    // ---------------------------------------------------------------------
    wire round_bit_stage1 = |(in_abs_stage1 & round_bit_mask);
    // roundToIntegralTiesToAway: round when the round bit is set.
    wire to_round_stage1  = round_bit_stage1;

    wire signed [MaxPrecision-1:0] in_abs_rounded_d2 =
        (in_abs_stage1 & ~(round_bit_mask|sticky_bits_mask)) + (to_round_stage1 ? guard_bit_mask : 0);

    wire                           in_sign_d2  = in_sign_stage1;
    wire                           to_clamp_d2 = |(in_abs_rounded_d2 & clamping_bits_mask);

    // Optional register after the round add.
    logic signed [MaxPrecision-1:0] in_abs_rounded_q2;
    logic                           in_sign_q2;
    logic                           to_clamp_q2;
    if (NumPipelineStages >= 2) begin : g_round_reg
        always_ff @(posedge clk_i) begin
            if (advance) begin
                in_abs_rounded_q2 <= in_abs_rounded_d2;
                in_sign_q2        <= in_sign_d2;
                to_clamp_q2       <= to_clamp_d2;
            end
            `ifndef SYNTHESIS
            if (!rst_ni) begin
                in_abs_rounded_q2 <= 'x;
                in_sign_q2        <= 'x;
                to_clamp_q2       <= 'x;
            end
            `endif
        end
    end else begin : g_round_comb
        assign in_abs_rounded_q2 = in_abs_rounded_d2;
        assign in_sign_q2        = in_sign_d2;
        assign to_clamp_q2       = to_clamp_d2;
    end

    // Value at stage 2 (registered when NumPipelineStages >= 2, else passthrough).
    wire signed [MaxPrecision-1:0]  in_abs_rounded_stage2 = in_abs_rounded_q2;
    wire                            in_sign_stage2        = in_sign_q2;
    wire                            to_clamp_stage2       = to_clamp_q2;

    // ---------------------------------------------------------------------
    // Stage 2 (combinational): shift, restore sign, clamp.
    // ---------------------------------------------------------------------
    logic signed [MaxPrecision-1:0] shifted_abs;
    always_comb begin
        if (LeftShiftAmount >= 0) begin
            shifted_abs = in_abs_rounded_stage2 <<< LeftShiftAmount;
        end else begin
            shifted_abs = in_abs_rounded_stage2 >>> -LeftShiftAmount;
        end

        // Restore sign
        out = in_sign_stage2 ? -shifted_abs : shifted_abs;
        if (to_clamp_stage2) begin
            out = (in_sign_stage2) ? OutMin : OutMax;
        end
    end

    `ifndef SYNTHESIS
    // only run assertions if within real precision
    if (MaxPrecision <= 52) begin : g_assert
        function automatic out_t convert_model(input logic signed [InPrecision-1:0] in_sample);
            automatic real out_max_real = (2.0**(OutPrecision-1)-1) * 2.0**OutExponent;
            automatic real out_min_real = -(2.0**(OutPrecision-1)) * 2.0**OutExponent;
            automatic real in_real = $itor(in_sample) * 2.0**InExponent;
            automatic real out_model_real = in_real;
            if (out_model_real < out_min_real) out_model_real = out_min_real;
            if (out_model_real > out_max_real) out_model_real = out_max_real;
            return out_t'(out_model_real / 2.0**OutExponent);
        endfunction

        // Delay the golden model by the same number of enabled register stages the
        // datapath uses, so the comparison stays aligned under pipelining / stall.
        out_t model_pipe [NumPipelineStages+1];
        logic known_pipe [NumPipelineStages+1];

        always_comb begin
            model_pipe[0] = convert_model(in);
            known_pipe[0] = !$isunknown(in) && rst_ni;
        end

        for (genvar s = 0; s < NumPipelineStages; s++) begin : g_model_delay
            always_ff @(posedge clk_i) begin
                if (!rst_ni) begin
                    known_pipe[s+1] <= 0;
                end else if (advance) begin
                    model_pipe[s+1] <= model_pipe[s];
                    known_pipe[s+1] <= known_pipe[s];
                end
            end
        end

        always @(posedge clk_i) #2ps begin
            if (rst_ni && known_pipe[NumPipelineStages] && !$isunknown(out)) begin
                if (model_pipe[NumPipelineStages] !== out) begin
                    $display(">>> CONVERSION FAILED at time %0t <<<", $time);
                    $display("    out_model (expected) = %0d  bin=%b  out (received) = %0d  bin=%b",
                             model_pipe[NumPipelineStages], model_pipe[NumPipelineStages], out, out);
                    $display("  Parameters:");
                    $display("    InPrecision=%0d  InExponent=%0d  OutPrecision=%0d  OutExponent=%0d",
                             InPrecision, InExponent, OutPrecision, OutExponent);
                    $display("    NumPipelineStages=%0d", NumPipelineStages);
                    assert(0) else $fatal(0, "ternip_fixed_point_convert output does not match expected value");
                end
            end
        end
    end
    `endif

endmodule
