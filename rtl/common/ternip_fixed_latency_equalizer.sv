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
//
// Only one transaction may be in flight: the caller must gate its own input
// handshake with idle_o, or the window restarts mid-flight and the latency stops
// being fixed. Asserted below rather than enforced structurally.

module ternip_fixed_latency_equalizer #(
    parameter int DataWidth = 16,
    parameter int NumCycles = 32
) (
    input  logic                 clk_i,
    input  logic                 rst_ni,

    output logic                 idle_o,
    input  logic                 in_accepted_i,

    input  logic                 unequalized_result_valid_i,
    output logic                 unequalized_result_ready_o,
    input  logic [DataWidth-1:0] unequalized_result_data_i,

    output logic                 equalized_result_valid_o,
    input  logic                 equalized_result_ready_i,
    output logic [DataWidth-1:0] equalized_result_data_o
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

assign idle_o                  = (state_q == WAITING_FOR_IN);
assign equalized_result_data_o = data_q;

always_comb begin
    state_d   = state_q;
    counter_d = counter_q;
    data_d    = data_q;

    unequalized_result_ready_o = 1;
    equalized_result_valid_o   = 0;

    if (state_q == WAITING_FOR_IN) begin

        if (in_accepted_i) begin
            state_d   = WAITING_FOR_RESULT;
            counter_d = '0;
        end

    end else if (state_q == WAITING_FOR_RESULT) begin

        if (!deadline_reached)
            counter_d++;

        // Drain the core exactly once, when its result first becomes valid.
        if (unequalized_result_valid_i && unequalized_result_ready_o) begin
            data_d  = unequalized_result_data_i;
            state_d = HOLDING_RESULT;
        end

    end else if (state_q == HOLDING_RESULT) begin
        unequalized_result_ready_o = 0;

        if (deadline_reached)
            equalized_result_valid_o = 1;
        else
            counter_d++;

        if (equalized_result_valid_o && equalized_result_ready_i)
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
// A core that misses the deadline makes the latency data-dependent again, which
// means NumCycles is too small for it. Sampled on the clock rather than checked
// inside the always_comb, so a transient evaluation cannot trip it.
always @(posedge clk_i) if (rst_ni) begin
    assert (!(state_q == WAITING_FOR_RESULT && deadline_reached))
        else $fatal(0, "core did not produce a result within %0d cycles", NumCycles);
end

// The whole point of this module is that latency does not depend on the data, so
// check it end to end rather than trusting the state machine not to add a cycle.
property fixed_latency_p;
    @(posedge clk_i) disable iff (!rst_ni)
    in_accepted_i |-> ##NumCycles equalized_result_valid_o;
endproperty

assert property (fixed_latency_p)
    else $fatal(0, "latency deviated from the fixed %0d cycles", NumCycles);

// The caller owns the one-in-flight rule, so check that it kept it.
property one_in_flight_p;
    @(posedge clk_i) disable iff (!rst_ni)
    in_accepted_i |-> idle_o;
endproperty

assert property (one_in_flight_p)
    else $fatal(0, "input accepted while a transaction was still in flight");
`endif

endmodule
