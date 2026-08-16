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

// ternip_multioperand_accumulator
//
// Pipelined reducer plus running accumulator for many signed operands.
//
// Each accepted input beat reduces NUM_OPERANDS values through a tree with
// NEXT_STAGE_FANIN operands per node, then adds the reduced value into an
// accumulator. Assert in_final_i on the last beat of an accumulation group; the
// clipped result is pushed to the output FIFO after the pipeline drains.
//
// Use this for dot-product or sum-of-products style reductions where many
// partial values arrive per cycle. NEXT_STAGE_FANIN must be a power of two.
// Backpressure is applied at in_ready_o when the output register/FIFO cannot
// accept another completed accumulation.

`define SIGN_EXTEND(X, FROM, TO) (TO'({ {$bits(TO){X[$bits(FROM)-1]}}, X }))

module ternip_multioperand_accumulator #(
    parameter ternip_pkg::ternip_cfg_t Cfg = `TERNIP_CFG,

    parameter type operand_t = logic signed [8:0],
    parameter type result_t = ternip_types#(Cfg)::fixed_point_t,
    parameter int NUM_OPERANDS = 16,
    parameter int NEXT_STAGE_FANIN = 4,
    parameter int D = Cfg.D
) (
    input  logic                        clk_i,
    input  logic                        rst_ni,

    output logic                        in_ready_o,
    input  logic                        in_valid_i,
    input  logic                        in_final_i,
    input  operand_t [NUM_OPERANDS-1:0] in_operands_i,

    input  logic                        out_ready_i,
    output logic                        out_valid_o,
    output result_t                     out_result_o
);

`ifndef SYNTHESIS
if (!$onehot(NEXT_STAGE_FANIN))
    $fatal(0, "NEXT_STAGE_FANIN must be a power of 2");
`endif

localparam int NumStages = (NUM_OPERANDS > 1) ? int'($ceil($ln(1.0*NUM_OPERANDS) / $ln(1.0*NEXT_STAGE_FANIN))) : 1;
localparam int OperandCapacity = NEXT_STAGE_FANIN**(NumStages-1);
localparam int PipelineDataWidth = ((NEXT_STAGE_FANIN-1) * NumStages) + $bits(operand_t);
localparam int AccumulatorWidth = PipelineDataWidth + $clog2(D);

typedef logic signed [PipelineDataWidth-1:0] pipeline_data_t;
typedef logic signed [AccumulatorWidth-1:0] accumulator_t;

localparam accumulator_t AccumulatorMin = (1 << (AccumulatorWidth-1));
localparam accumulator_t AccumulatorMax = (AccumulatorMin - 1);

function automatic int NumOperandsPerStage(int stage);
    if (stage == 0) return NUM_OPERANDS;
    return (NEXT_STAGE_FANIN**(NumStages-stage));
endfunction

// final_pipeline: NumStages+1 bits — tracks in_final_i; extra bit lets last data accumulate
// valid_pipeline: NumStages bits — tracks in_valid_i, aligned with data_q[NumStages-1]
logic [NumStages:0] final_pipeline_d, final_pipeline_q;
logic [NumStages-1:0] valid_pipeline_d, valid_pipeline_q;
pipeline_data_t [NumStages-1:0][OperandCapacity-1:0] data_d, data_q;
accumulator_t accumulator_d, accumulator_q;

enum logic [1:0] {
    STARTING_FLUSH,
    FLUSHING,
    WORKING
} state_d, state_q;

wire pipeline_data_valid = valid_pipeline_q[NumStages-1];

// Output FIFO signals
localparam logic ResultIsSigned = ( result_t'('1) < 0 );
localparam result_t ResultMin = ResultIsSigned ? (1 << ($bits(result_t)-1)) : ('0);
localparam result_t ResultMax = ResultMin-1;

result_t clipped_result;
always_comb begin
    clipped_result = accumulator_q;
    if (accumulator_q > ResultMax) clipped_result = ResultMax;
    if (accumulator_q < ResultMin) clipped_result = ResultMin;
end

logic                       fifo_in_ready;
logic                       fifo_in_valid;
logic [$bits(result_t)-1:0] fifo_in_data;

logic                       fifo_out_ready;
logic                       fifo_out_valid;
logic [$bits(result_t)-1:0] fifo_out_data;

assign fifo_in_data = clipped_result;

// Registered output stage — breaks combinational path from FIFO async-read memory to out_result_o.
// fifo_out_ready: pop from FIFO when the output register is empty or being consumed this cycle.
logic    out_valid_d, out_valid_q;
result_t out_result_d, out_result_q;

assign fifo_out_ready = !out_valid_q || out_ready_i;

assign out_valid_o = out_valid_q;
always_comb begin
    out_result_o = out_result_q;
    if (!out_valid_q) out_result_o = 'x;
end

always_comb begin
    out_valid_d = out_valid_q;
    if (fifo_out_ready)
        out_valid_d = fifo_out_valid;
end
always_comb begin
    out_result_d = out_result_q;
    if (fifo_out_ready && fifo_out_valid)
        out_result_d = result_t'(fifo_out_data);
end

// State and control
always_comb begin
    state_d = state_q;
    in_ready_o = 0;
    fifo_in_valid = 0;
    valid_pipeline_d = {valid_pipeline_q, 1'b0};
    final_pipeline_d = {final_pipeline_q, 1'b0};

    if (state_q == STARTING_FLUSH) begin
        final_pipeline_d = {final_pipeline_q, 1'b1};
        state_d = FLUSHING;
    end else if (state_q == FLUSHING) begin
        if (final_pipeline_q[NumStages])
            state_d = WORKING;
    end else if (state_q == WORKING) begin
        in_ready_o = !out_valid_q;
        fifo_in_valid = final_pipeline_q[NumStages];
        valid_pipeline_d = {valid_pipeline_q, (in_ready_o && in_valid_i)};
        final_pipeline_d = {final_pipeline_q, (in_ready_o && in_valid_i && in_final_i)};
    end
end

// Data path — always computes, no stalling
always_comb begin
    data_d = '0;
    for (int stage = 0; stage < NumStages; stage++) begin
        for (int operand = 0; operand < NumOperandsPerStage(stage); operand++) begin
            if (stage == 0) begin
                // Dodging Yosys Bug
                data_d[stage][operand / NEXT_STAGE_FANIN] += `SIGN_EXTEND(in_operands_i[operand], operand_t, pipeline_data_t);
                // data_d[stage][operand / NEXT_STAGE_FANIN] += in_operands_i[operand];
            end else begin
                data_d[stage][operand / NEXT_STAGE_FANIN] += data_q[stage-1][operand];
            end
        end
    end
end

// Accumulator — gated by pipeline_data_valid at the narrow output
typedef logic signed [AccumulatorWidth:0] unclipped_t;
always_comb begin
    automatic pipeline_data_t pipeline_addend = pipeline_data_valid ? data_q[NumStages-1][0] : '0;
    automatic unclipped_t unclipped = `SIGN_EXTEND(pipeline_addend, pipeline_data_t, unclipped_t);
    // final_pipeline_q[NumStages] clears accumulator_q for the next accumulation
    if (!final_pipeline_q[NumStages]) unclipped += `SIGN_EXTEND(accumulator_q, accumulator_t, unclipped_t);
    accumulator_d = unclipped;
    if (unclipped > AccumulatorMax) accumulator_d = AccumulatorMax;
    if (unclipped < AccumulatorMin) accumulator_d = AccumulatorMin;
end

// Pipeline registers — always advance, no stall
always_ff @(posedge clk_i) begin
    if (!rst_ni) begin
        final_pipeline_q <= '0;
        valid_pipeline_q <= '0;
        state_q <= STARTING_FLUSH;
    end else begin
        final_pipeline_q <= final_pipeline_d;
        valid_pipeline_q <= valid_pipeline_d;
        state_q <= state_d;
    end
end
always_ff @(posedge clk_i) begin
    data_q <= data_d;
    accumulator_q <= accumulator_d;
    `ifndef SYNTHESIS
    if (!rst_ni) begin
        data_q <= 'x;
        accumulator_q <= 'x;
    end
    `endif
end

// Output register flip-flops
always_ff @(posedge clk_i) begin
    if (!rst_ni)
        out_valid_q <= 1'b0;
    else
        out_valid_q <= out_valid_d;
end
always_ff @(posedge clk_i) begin
    out_result_q <= out_result_d;
    `ifndef SYNTHESIS
    if (!rst_ni) out_result_q <= 'x;
    `endif
end

// https://github.com/bespoke-silicon-group/basejump_stl/blob/a43571d2/bsg_dataflow/bsg_fifo_1r1w_small.sv
bsg_fifo_1r1w_small #(
    .width_p($bits(result_t)),
    .els_p(NumStages + 1),
    .harden_p(0),
    .ready_THEN_valid_p(0)
) output_fifo (
    .clk_i,
    .reset_i(!rst_ni),

    .v_i(fifo_in_valid),
    .ready_param_o(fifo_in_ready),
    .data_i(fifo_in_data),

    .v_o(fifo_out_valid),
    .data_o(fifo_out_data),
    .yumi_i(fifo_out_ready && fifo_out_valid)
);

assert property (@(posedge clk_i) disable iff (!rst_ni) fifo_in_valid |-> fifo_in_ready)
    else $fatal(0, "Output FIFO overflow");

endmodule

`undef SIGN_EXTEND
