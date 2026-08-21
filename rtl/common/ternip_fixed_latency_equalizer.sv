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


// ternip_fixed_latency_equalizer
//
// Gives a variable-latency ready/valid core a fixed, data-independent latency.

module ternip_fixed_latency_equalizer #(
    parameter int DataWidth = 16,
    parameter int NumCycles = 32
) (
    input  logic                 clk_i,
    input  logic                 rst_ni,

    output logic                 in_ready_o,
    input  logic                 in_valid_i,

    input  logic                 wrappedcore_in_ready_i,
    output logic                 wrappedcore_in_valid_o,

    output logic                 wrappedcore_out_ready_o,
    input  logic                 wrappedcore_out_valid_i,
    input  logic [DataWidth-1:0] wrappedcore_out_data_i,

    input  logic                 out_ready_i,
    output logic                 out_valid_o,
    output logic [DataWidth-1:0] out_data_o
);

localparam int CounterWidth = $clog2(NumCycles);

enum logic [1:0] {
    WAITING_FOR_IN,
    WAITING_FOR_RESULT,
    HOLDING_RESULT
} state_d, state_q = WAITING_FOR_IN; // for assertions at time=0

logic [CounterWidth-1:0] counter_d, counter_q;
logic [DataWidth-1:0]    data_d, data_q;

wire deadline_reached = (counter_q >= CounterWidth'(NumCycles-1));

assign out_data_o = data_q;

always_comb begin
    state_d   = state_q;
    counter_d = counter_q;
    data_d    = data_q;

    in_ready_o              = 0;
    wrappedcore_in_valid_o  = 0;
    wrappedcore_out_ready_o = 1;
    out_valid_o             = 0;

    // Only pass the input handshake through while idle, so a second transaction
    // can never restart the window and cost the latency its independence.
    if (state_q == WAITING_FOR_IN) begin

        in_ready_o             = wrappedcore_in_ready_i;
        wrappedcore_in_valid_o = in_valid_i;

        if (in_ready_o && in_valid_i) begin
            state_d   = WAITING_FOR_RESULT;
            counter_d = '0;
        end

    end else if (state_q == WAITING_FOR_RESULT) begin

        counter_d++; // assuming deadline_reached==0
`ifndef SYNTHESIS
        if (deadline_reached)
            $fatal(0, "core did not produce a result within %0d cycles", NumCycles);
`endif

        if (wrappedcore_out_ready_o && wrappedcore_out_valid_i) begin
            data_d  = wrappedcore_out_data_i;
            state_d = HOLDING_RESULT;
        end

    end else if (state_q == HOLDING_RESULT) begin
        wrappedcore_out_ready_o = 0;

        if (deadline_reached)
            out_valid_o = 1;
        else
            counter_d++;

        if (out_ready_i && out_valid_o)
            state_d = WAITING_FOR_IN;

    end else begin
        state_d = WAITING_FOR_IN;
    end
end

always_ff @(posedge clk_i) begin
    if (!rst_ni) begin
        state_q   <= WAITING_FOR_IN;
        counter_q <= '0;
        data_q    <= '0;
    end else begin
        state_q   <= state_d;
        counter_q <= counter_d;
        data_q    <= data_d;
    end
end

`ifndef SYNTHESIS

// Ensure that correct latency is achieved
property fixed_latency_p;
    @(posedge clk_i) disable iff (!rst_ni)
    (in_ready_o && in_valid_i) |-> ##NumCycles out_valid_o;
endproperty

assert property (fixed_latency_p)
    else $fatal(0, "latency deviated from the fixed %0d cycles", NumCycles);

`endif

endmodule
