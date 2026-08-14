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

// ternip_mul
//
// Signed fixed-point multiply.
//
// Multiplies a_i and b_i in a wider internal fixed-point format, then converts
// and saturates the product to OutPrecision/OutExponent. Implementation selects
// BSG, round-robin, starmul, or placeholder behavior.

module ternip_mul #(
    parameter ternip_pkg::ternip_cfg_t Cfg = `TERNIP_CFG,

    parameter int InAPrecision = Cfg.FixedPointPrecision,
    parameter int InAExponent  = Cfg.FixedPointExponent,
    parameter int InBPrecision = Cfg.FixedPointPrecision,
    parameter int InBExponent  = Cfg.FixedPointExponent,
    parameter int OutPrecision = Cfg.FixedPointPrecision,
    parameter int OutExponent  = Cfg.FixedPointExponent,
    parameter ternip_pkg::mul_impl_e Implementation = Cfg.MultiplicationImplementation,
    parameter int FinalConversionNumPipelineStages = 0
) (
    input  logic                           clk_i,
    input  logic                           rst_ni,

    output logic                           in_ready_o,
    input  logic                           in_valid_i,
    input  logic signed [InAPrecision-1:0] a_i,
    input  logic signed [InBPrecision-1:0] b_i,

    input  logic                           out_ready_i,
    output logic                           out_valid_o,
    output logic signed [OutPrecision-1:0] y_o
);

localparam int MulInternalPrecision = InAPrecision + InBPrecision;
localparam int PostMulInternalExponent = InAExponent + InBExponent;

logic signed [MulInternalPrecision-1:0] internal_a;
logic signed [MulInternalPrecision-1:0] internal_b;
logic signed [MulInternalPrecision-1:0] internal_y;

logic mul_out_valid;
logic mul_out_ready;

ternip_fixed_point_convert #(
    .InPrecision(InAPrecision),
    .InExponent(InAExponent),
    .OutPrecision(MulInternalPrecision),
    .OutExponent(InAExponent)
) convert_a (
    .clk_i,
    .rst_ni,
    .in_valid_i(1'b1),
    .in_ready_o(),
    .out_valid_o(),
    .out_ready_i(1'b1),
    .in(a_i),
    .out(internal_a)
);

ternip_fixed_point_convert #(
    .InPrecision(InBPrecision),
    .InExponent(InBExponent),
    .OutPrecision(MulInternalPrecision),
    .OutExponent(InBExponent)
) convert_b (
    .clk_i,
    .rst_ni,
    .in_valid_i(1'b1),
    .in_ready_o(),
    .out_valid_o(),
    .out_ready_i(1'b1),
    .in(b_i),
    .out(internal_b)
);

ternip_fixed_point_convert #(
    .InPrecision(MulInternalPrecision),
    .InExponent(PostMulInternalExponent),
    .OutPrecision(OutPrecision),
    .OutExponent(OutExponent),
    .NumPipelineStages(FinalConversionNumPipelineStages)
) convert_y (
    .clk_i,
    .rst_ni,
    .in(internal_y),
    .in_valid_i(mul_out_valid),
    .in_ready_o(mul_out_ready),
    .out(y_o),
    .out_valid_o(out_valid_o),
    .out_ready_i(out_ready_i)
);

if (Implementation == ternip_pkg::MUL_BSG) begin : mul_bsg

    // https://github.com/bespoke-silicon-group/basejump_stl/blob/a43571d2/bsg_misc/bsg_imul_iterative.sv
    bsg_imul_iterative #(
        .width_p(MulInternalPrecision)
    ) bsg_imul_iterative (
        .clk_i,
        .reset_i(!rst_ni),
        .v_i(in_valid_i),
        .ready_and_o(in_ready_o),

        .opA_i(internal_a),
        .signed_opA_i(1),
        .opB_i(internal_b),
        .signed_opB_i(1),
        .gets_high_part_i(0),

        .v_o(mul_out_valid),
        .result_o(internal_y),
        .yumi_i(mul_out_ready && mul_out_valid)
    );

end else if (Implementation == ternip_pkg::MUL_ROUNDROBIN) begin : mul_roundrobin

    ternip_round_robin_operation #(
        .DataWidth(MulInternalPrecision),
        .NumRobins(16),
        .Operation("MUL")
    ) integer_multiplier (
        .clk_i,
        .rst_ni,

        .in_ready_o,
        .in_valid_i,
        .a_i(internal_a),
        .b_i(internal_b),

        .out_ready_i(mul_out_ready),
        .out_valid_o(mul_out_valid),
        .out_div_remainder_o(),
        .y_o(internal_y)
    );

end else if (Implementation == ternip_pkg::MUL_STAR) begin : mul_star

    ternip_starmul #(
        .DataWidth(MulInternalPrecision)
    ) starmul (
        .clk_i,
        .rst_ni,

        .in_ready_o,
        .in_valid_i,
        .a_i(internal_a),
        .b_i(internal_b),

        .out_ready_i(mul_out_ready),
        .out_valid_o(mul_out_valid),
        .y_o(internal_y)
    );

end else begin

    assign in_ready_o = 1;
    assign mul_out_valid = 1;

end

`ifndef SYNTHESIS

function automatic logic signed [OutPrecision-1:0] mul_model(logic signed [InAPrecision-1:0] a, logic signed [InBPrecision-1:0] b);
    typedef logic signed [OutPrecision-1:0] out_t;
    real a_real = $itor(a) * (2.0 ** InAExponent);
    real b_real = $itor(b) * (2.0 ** InBExponent);
    real y_unclamped_real = a_real * b_real;
    real out_real = y_unclamped_real;
    logic signed [OutPrecision-1:0] OutMin = (1 << (OutPrecision-1));
    logic signed [OutPrecision-1:0] OutMax = (OutMin - 1);

    if (out_real < ($itor(OutMin) * (2.0 ** OutExponent)))
        out_real = ($itor(OutMin) * (2.0 ** OutExponent));
    if (out_real > ($itor(OutMax) * (2.0 ** OutExponent)))
        out_real = ($itor(OutMax) * (2.0 ** OutExponent));
    out_real /= (2.0 ** OutExponent);

    return out_t'(out_real);
endfunction

typedef struct {
    logic signed [InAPrecision-1:0] a;
    logic signed [InBPrecision-1:0] b;
    logic signed [MulInternalPrecision-1:0] internal_a;
    logic signed [MulInternalPrecision-1:0] internal_b;
    logic signed [OutPrecision-1:0] expected_y;
} input_t;

input_t expected_queue [$];

always @(posedge clk_i) #2ps begin
    while (!rst_ni || !(in_ready_o && in_valid_i)) begin
        @(posedge clk_i); #2ps;
    end

    expected_queue.push_back(input_t'{a:a_i,
                                      b:b_i,
                                      internal_a:internal_a,
                                      internal_b:internal_b,
                                      expected_y:mul_model(a_i, b_i)});
end

// only run assertions if within real precision
if (Implementation != ternip_pkg::MUL_NONE) if (MulInternalPrecision <= 52) always @(posedge clk_i) #2ps begin
    while (!rst_ni || !(out_ready_i && out_valid_o)) begin
        @(posedge clk_i); #2ps;
    end

    assert (expected_queue[0].expected_y == y_o) else begin
        $display("======== FAILING MULTIPLICATION ========");
        $display("           InAPrecision=%0d", InAPrecision);
        $display("            InAExponent=%0d", InAExponent);
        $display("           InBPrecision=%0d", InBPrecision);
        $display("            InBExponent=%0d", InBExponent);
        $display("           OutPrecision=%0d", OutPrecision);
        $display("            OutExponent=%0d", OutExponent);
        $display("   MulInternalPrecision=%0d", MulInternalPrecision);
        $display("PostMulInternalExponent=%0d", PostMulInternalExponent);
        $display("       a_i = %b (%0f)", expected_queue[0].a, $itor(expected_queue[0].a) * 2.0**InAExponent);
        $display("       b_i = %b (%0f)", expected_queue[0].b, $itor(expected_queue[0].b) * 2.0**InBExponent);
        $display("internal_a = %b (%0f)", expected_queue[0].internal_a, $itor(expected_queue[0].internal_a) * 2.0**InAExponent);
        $display("internal_b = %b (%0f)", expected_queue[0].internal_b, $itor(expected_queue[0].internal_b) * 2.0**InAExponent);
        $display("internal_y = %b %0d (%0f)", internal_y, internal_y, $itor(internal_y) * 2.0**PostMulInternalExponent);
        $display("       y_o = %b (%0f)", y_o, $itor(y_o) * 2.0**OutExponent);
        $display("expected_y = %b (%0f)", expected_queue[0].expected_y, $itor(expected_queue[0].expected_y) * 2.0**OutExponent);
        $fatal;
    end
    expected_queue.pop_front();
end
`endif

endmodule
