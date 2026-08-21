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

// ternip_div
//
// Signed fixed-point divide.
//
// Computes a_i / b_i by widening the operands, shifting the dividend to keep
// fractional bits, and converting the quotient to OutPrecision/OutExponent with
// saturation. Implementation selects BSG, round-robin, or placeholder behavior.

module ternip_div #(
    parameter ternip_pkg::ternip_cfg_t Cfg = `TERNIP_CFG,

    parameter int InAPrecision = Cfg.FixedPointPrecision,
    parameter int InAExponent  = Cfg.FixedPointExponent,
    parameter int InBPrecision = Cfg.FixedPointPrecision,
    parameter int InBExponent  = Cfg.FixedPointExponent,
    parameter int OutPrecision = Cfg.FixedPointPrecision,
    parameter int OutExponent  = Cfg.FixedPointExponent,
    parameter ternip_pkg::div_impl_e Implementation = Cfg.DivisionImplementation
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

localparam int ALeftShiftAmount = InBPrecision + 2;
localparam int DivInternalPrecision = InAPrecision + ALeftShiftAmount + 1
    + ((Implementation==ternip_pkg::DIV_BSG)&&((InAPrecision+ALeftShiftAmount+1)%2));
    // ensure DivInternalPrecision is even for bsg_idiv_iterative.bits_per_iter_p=2

localparam int InternalAInternalExponent = InAExponent - ALeftShiftAmount;
localparam int InternalBInternalExponent = InBExponent;
localparam int InternalYInternalExponent = InternalAInternalExponent - InternalBInternalExponent;

logic signed [DivInternalPrecision-1:0] internal_a;
logic signed [DivInternalPrecision-1:0] internal_b;
logic signed [DivInternalPrecision-1:0] internal_y;

logic                           raw_in_ready;
logic                           raw_in_valid;
logic                           div_out_valid;
logic                           div_out_ready;
logic signed [OutPrecision-1:0] convert_out_y;

ternip_fixed_point_convert #(
    .InPrecision(InAPrecision),
    .InExponent(InAExponent),
    .OutPrecision(DivInternalPrecision),
    .OutExponent(InternalAInternalExponent),
    .NumPipelineStages(0)
) convert_a (
    .clk_i, .rst_ni, .in_ready_o(), .in_valid_i(1'b1), .out_ready_i(1'b1), .out_valid_o(), // unused, module combinational
    .in(a_i),
    .out(internal_a)
);

ternip_fixed_point_convert #(
    .InPrecision(InBPrecision),
    .InExponent(InBExponent),
    .OutPrecision(DivInternalPrecision),
    .OutExponent(InternalBInternalExponent),
    .NumPipelineStages(0)
) convert_b (
    .clk_i, .rst_ni, .in_ready_o(), .in_valid_i(1'b1), .out_ready_i(1'b1), .out_valid_o(), // unused, module combinational
    .in(b_i),
    .out(internal_b)
);

ternip_fixed_point_convert #(
    .InPrecision(DivInternalPrecision),
    .InExponent(InternalYInternalExponent),
    .OutPrecision(OutPrecision),
    .OutExponent(OutExponent),
    .NumPipelineStages(0)
) convert_out (
    .clk_i, .rst_ni, .in_ready_o(), .in_valid_i(1'b1), .out_ready_i(1'b1), .out_valid_o(), // unused, module combinational
    .in(internal_y),
    .out(convert_out_y)
);

// Fixed-latency equalizer: the iterative divider takes a data-dependent number of
// cycles, long enough that every divider in all cores is done
localparam int RawDivideLatencyUpperBound = DivInternalPrecision + 16;

ternip_fixed_latency_equalizer #(
    .DataWidth(OutPrecision),
    .NumCycles(RawDivideLatencyUpperBound + 1)
) equalizer (
    .clk_i,
    .rst_ni,

    .in_ready_o,
    .in_valid_i,

    .wrappedcore_in_ready_i(raw_in_ready),
    .wrappedcore_in_valid_o(raw_in_valid),

    .wrappedcore_out_ready_o(div_out_ready),
    .wrappedcore_out_valid_i(div_out_valid),
    .wrappedcore_out_data_i(convert_out_y),

    .out_ready_i,
    .out_valid_o,
    .out_data_o(y_o)
);

if (Implementation == ternip_pkg::DIV_BSG) begin : div_bsg

    // https://github.com/bespoke-silicon-group/basejump_stl/blob/a43571d2/bsg_misc/bsg_idiv_iterative.sv
    bsg_idiv_iterative #(
        .width_p(DivInternalPrecision),
        .bits_per_iter_p(1)
    ) bsg_idiv_iterative (
        .clk_i,
        .reset_i(!rst_ni),
        .v_i(raw_in_valid),
        .ready_and_o(raw_in_ready),

        .dividend_i(internal_a),
        .divisor_i(internal_b),
        .signed_div_i(1),

        .v_o(div_out_valid),
        .quotient_o(internal_y),
        .remainder_o(),
        .yumi_i(div_out_ready && div_out_valid)
    );

end else if (Implementation == ternip_pkg::DIV_ROUNDROBIN) begin : div_roundrobin

    ternip_round_robin_operation #(
        .DataWidth(DivInternalPrecision),
        .NumRobins(16),
        .Operation("DIV")
    ) integer_divider (
        .clk_i,
        .rst_ni,
        .in_ready_o(raw_in_ready),
        .in_valid_i(raw_in_valid),
        .a_i(internal_a),
        .b_i(internal_b),
        .out_ready_i(div_out_ready),
        .out_valid_o(div_out_valid),
        .out_div_remainder_o(),
        .y_o(internal_y)
    );

end else begin

    assign raw_in_ready = 1;
    assign div_out_valid = 1;
    assign internal_y = 'x;

end

`ifndef SYNTHESIS

function automatic logic signed [OutPrecision-1:0] div_model(logic signed [InAPrecision-1:0] a, logic signed [InBPrecision-1:0] b);
    typedef logic signed [OutPrecision-1:0] out_t;
    real a_real = $itor(a) * (2.0 ** InAExponent);
    real b_real = $itor(b) * (2.0 ** InBExponent);
    real y_unclamped_real = a_real / b_real;
    logic signed [OutPrecision-1:0] OutMin = (1 << (OutPrecision-1));
    logic signed [OutPrecision-1:0] OutMax = (OutMin - 1);

    real out_real = y_unclamped_real;
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
    logic signed [DivInternalPrecision-1:0] internal_a;
    logic signed [DivInternalPrecision-1:0] internal_b;
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
                                      expected_y:div_model(a_i, b_i)});
end

// only run assertions if within real precision
if (Implementation != ternip_pkg::DIV_BSG) if (DivInternalPrecision <= 52) always @(posedge clk_i) #2ps begin
    while (!rst_ni || !(out_ready_i && out_valid_o)) begin
        @(posedge clk_i); #2ps;
    end

    assert (expected_queue[0].b == 0 || expected_queue[0].expected_y == y_o) else begin
        $display("======== FAILING DIVISION ========");
        $display("             InAPrecision=%0d", InAPrecision);
        $display("              InAExponent=%0d", InAExponent);
        $display("             InBPrecision=%0d", InBPrecision);
        $display("              InBExponent=%0d", InBExponent);
        $display("             OutPrecision=%0d", OutPrecision);
        $display("              OutExponent=%0d", OutExponent);
        $display("         ALeftShiftAmount=%0d", ALeftShiftAmount);
        $display("     DivInternalPrecision=%0d", DivInternalPrecision);
        $display("InternalAInternalExponent=%0d", InternalAInternalExponent);
        $display("InternalBInternalExponent=%0d", InternalBInternalExponent);
        $display("InternalYInternalExponent=%0d", InternalYInternalExponent);
        $display("       a_i = %b (%0f)", expected_queue[0].a, $itor(expected_queue[0].a) * 2.0**InAExponent);
        $display("       b_i = %b (%0f)", expected_queue[0].b, $itor(expected_queue[0].b) * 2.0**InBExponent);
        $display("internal_a = %b %0d (%0f)", expected_queue[0].internal_a, expected_queue[0].internal_a, $itor(expected_queue[0].internal_a) * 2.0**InternalAInternalExponent);
        $display("internal_b = %b %0d (%0f)", expected_queue[0].internal_b, expected_queue[0].internal_b, $itor(expected_queue[0].internal_b) * 2.0**InternalBInternalExponent);
        $display("internal_y = %b %0d (%0f)", internal_y, internal_y, $itor(internal_y) * 2.0**InternalYInternalExponent);
        $display("       y_o = %b (%0f)", y_o, $itor(y_o) * 2.0**OutExponent);
        $display("expected_y = %b (%0f)", expected_queue[0].expected_y, $itor(expected_queue[0].expected_y) * 2.0**OutExponent);
    end
    expected_queue.pop_front();
end
`endif

endmodule
