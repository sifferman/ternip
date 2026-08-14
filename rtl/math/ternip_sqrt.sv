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

// ternip_sqrt
//
// Fixed-point square root.
//
// Converts a_i to an unsigned internal format, computes floor(sqrt(a_i)) with
// ternip_sqrt_int, and converts the result to OutPrecision/OutExponent. Inputs
// should be non-negative.

module ternip_sqrt #(
    parameter int InPrecision = 8,
    parameter int InExponent = -3,
    parameter int OutPrecision = 8,
    parameter int OutExponent = -3
) (
    input  logic                           clk_i,
    input  logic                           rst_ni,

    output logic                           in_ready_o,
    input  logic                           in_valid_i,
    input  logic signed [InPrecision-1:0]  a_i,

    input  logic                           out_ready_i,
    output logic                           out_valid_o,
    output logic signed [OutPrecision-1:0] y_o
);

localparam int ExtraGuardBits = 1;
localparam int PostSqrtInternalExponent = OutExponent - ExtraGuardBits;
localparam int PreSqrtInternalExponent  = 2 * PostSqrtInternalExponent;
localparam int SqrtInternalPrecision = ternip_pkg::max_int(InExponent - (PreSqrtInternalExponent - InPrecision), 1);

`ifndef SYNTHESIS
initial assert(SqrtInternalPrecision + PreSqrtInternalExponent >= InPrecision + InExponent);
initial assert(PreSqrtInternalExponent == 2*PostSqrtInternalExponent);
initial assert(PostSqrtInternalExponent <= OutExponent);
`endif

logic signed [SqrtInternalPrecision-1:0] internal_a;
logic signed [SqrtInternalPrecision-1:0] internal_y;

ternip_fixed_point_convert #(
    .InPrecision(InPrecision),
    .InExponent(InExponent),
    .OutPrecision(SqrtInternalPrecision),
    .OutExponent(PreSqrtInternalExponent)
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
    .InPrecision(SqrtInternalPrecision),
    .InExponent(PostSqrtInternalExponent),
    .OutPrecision(OutPrecision),
    .OutExponent(OutExponent)
) convert_y (
    .clk_i,
    .rst_ni,
    .in_valid_i(1'b1),
    .in_ready_o(),
    .out_valid_o(),
    .out_ready_i(1'b1),
    .in(internal_y),
    .out(y_o)
);

ternip_sqrt_int #(
    .Width(SqrtInternalPrecision)
) sqrt_int (
    .clk_i,
    .rst_ni,

    .in_ready_o,
    .in_valid_i,
    .a_i(internal_a),

    .out_ready_i,
    .out_valid_o,
    .y_o(internal_y)
);

`ifndef SYNTHESIS
function automatic logic signed [OutPrecision-1:0] sqrt_model(logic signed [InPrecision-1:0] a);
    real a_real = $itor(a) * 2.0**InExponent;

    typedef logic signed [OutPrecision-1:0] out_t;
    out_t OutMin = (1 << (OutPrecision-1));
    out_t OutMax = (OutMin - 1);
    real OutMinReal = $itor(OutMin) * 2.0**OutExponent;
    real OutMaxReal = $itor(OutMax) * 2.0**OutExponent;

    real y_model_real = $sqrt(a_real);

    real y_model_clipped_real = y_model_real;
    if (y_model_clipped_real < OutMinReal)
        y_model_clipped_real = OutMinReal;
    if (y_model_clipped_real > OutMaxReal)
        y_model_clipped_real = OutMaxReal;

    return out_t'(y_model_clipped_real / 2.0**OutExponent);
endfunction

function automatic logic signed [SqrtInternalPrecision-1:0] int_sqrt_model(logic signed [SqrtInternalPrecision-1:0] a);
    typedef logic signed [SqrtInternalPrecision-1:0] post_sqrt_internal_precision_t;
    return post_sqrt_internal_precision_t'($sqrt(real'(a)));
endfunction

typedef struct {
    logic signed [InPrecision-1:0] a;
    logic signed [SqrtInternalPrecision-1:0] internal_a;
    logic signed [SqrtInternalPrecision-1:0] expected_internal_y;
    logic signed [OutPrecision-1:0] expected_y;
} input_t;

input_t expected_queue [$];

always @(posedge clk_i) #2ps begin
    while (!rst_ni || !(in_ready_o && in_valid_i)) begin
        @(posedge clk_i); #2ps;
    end

    expected_queue.push_back(input_t'{a:a_i,
                                      internal_a:internal_a,
                                      expected_internal_y:int_sqrt_model(internal_a),
                                      expected_y:sqrt_model(a_i)});
end

// only run assertions if within real precision
if (SqrtInternalPrecision <= 52) always @(posedge clk_i) #2ps begin
    while (!rst_ni || !(out_ready_i && out_valid_o)) begin
        @(posedge clk_i); #2ps;
    end

    if (ternip_pkg::abs_int(expected_queue[0].expected_internal_y - internal_y) >> ExtraGuardBits) begin
        $display("======== FAILING INTEGER SQRT ========");
        $display("         internal_a = %b %0d", expected_queue[0].internal_a, expected_queue[0].internal_a);
        $display("         internal_y = %b %0d", internal_y, internal_y);
        $display("expected_internal_y = %b %0d", expected_queue[0].expected_internal_y, expected_queue[0].expected_internal_y);
        // $fatal;
    end else if (ternip_pkg::abs_int(expected_queue[0].expected_y - y_o) > 1) begin
        $display("======== FAILING SQRT ========");
        $display("               InPrecision=%0d", InPrecision);
        $display("                InExponent=%0d", InExponent);
        $display("              OutPrecision=%0d", OutPrecision);
        $display("               OutExponent=%0d", OutExponent);
        $display("  SqrtInternalPrecision=%0d", SqrtInternalPrecision);
        $display("   PreSqrtInternalExponent=%0d", PreSqrtInternalExponent);
        $display(" SqrtInternalPrecision=%0d", SqrtInternalPrecision);
        $display("  PostSqrtInternalExponent=%0d", PostSqrtInternalExponent);
        $display("       a_i = %b %0d (%0f)", expected_queue[0].a, expected_queue[0].a, $itor(expected_queue[0].a) * 2.0**InExponent);
        $display("internal_a = %b %0d (%0f)", expected_queue[0].internal_a, expected_queue[0].internal_a, $itor(expected_queue[0].internal_a) * 2.0**PreSqrtInternalExponent);
        $display("internal_y = %b %0d (%0f)", internal_y, internal_y, $itor(internal_y) * 2.0**PostSqrtInternalExponent);
        $display("       y_o = %b %0d (%0f)", y_o, y_o, $itor(y_o) * 2.0**OutExponent);
        $display("expected_y = %b %0d (%0f)", expected_queue[0].expected_y, expected_queue[0].expected_y, $itor(expected_queue[0].expected_y) * 2.0**OutExponent);
        $fatal;
    end
    expected_queue.pop_front();
end
`endif

endmodule
