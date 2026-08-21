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

    input  logic                 in_valid_i,
    output logic                 in_ready_o,

    output logic                 unequalized_in_valid_o,
    input  logic                 unequalized_in_ready_i,

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

assign equalized_result_data_o = data_q;

always_comb begin
    state_d   = state_q;
    counter_d = counter_q;
    data_d    = data_q;

    unequalized_in_valid_o     = 0;
    in_ready_o                 = 0;
    unequalized_result_ready_o = 1;
    equalized_result_valid_o   = 0;

    if (state_q == WAITING_FOR_IN) begin

        unequalized_in_valid_o = in_valid_i;
        in_ready_o             = unequalized_in_ready_i;

        if (in_valid_i && in_ready_o) begin
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

        // The core blew the deadline, so NumCycles is too small. Retire on
        // schedule regardless to keep lockstep peers aligned; the data is stale.
        // Deliberately does not wait for ready -- this path is fatal.
        if (state_d == WAITING_FOR_RESULT && deadline_reached) begin
            state_d                  = WAITING_FOR_IN;
            equalized_result_valid_o = 1;
`ifndef SYNTHESIS
            $fatal(0, "core did not produce a result within %0d cycles", NumCycles);
`endif
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
// The whole point of this module is that latency does not depend on the data, so
// check it end to end rather than trusting the state machine not to add a cycle.
property fixed_latency_p;
    @(posedge clk_i) disable iff (!rst_ni)
    (in_valid_i && in_ready_o) |-> ##NumCycles equalized_result_valid_o;
endproperty

assert property (fixed_latency_p)
    else $fatal(0, "latency deviated from the fixed %0d cycles", NumCycles);
`endif

endmodule
