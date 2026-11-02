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

// ternip_gearbox_fifo
//
// Ready/valid width converter between arbitrary input and output word sizes.
//
// If one width is an integer multiple of the other, this wraps the appropriate
// BaseJump PISO or SIPO primitive. For non-multiple widths it collects input
// words into a staging register, emits OutDataWidth chunks, and preserves little
// endian bit order by shifting low bits out first.
//
// Use this to bridge stream widths such as byte streams to vector chunks.

`define SAFE_CLOG2(x) ( (((x)==1) || ((x)==0))? 1 : $clog2((x)))

module ternip_gearbox_fifo #(
    parameter int InDataWidth = 8,
    parameter int OutDataWidth = 8
) (
    input  logic                    clk_i,
    input  logic                    rst_ni,

    output logic                    in_ready_o,
    input  logic                    in_valid_i,
    input  logic [InDataWidth-1:0]  in_data_i,

    input  logic                    out_ready_i,
    output logic                    out_valid_o,
    output logic [OutDataWidth-1:0] out_data_o
);

function automatic int gcd(int a, int b);
    while (b != 0) begin
    int t = a % b;
    a = b;
    b = t;
    end
    return a;
endfunction

if (gcd(InDataWidth, OutDataWidth) == OutDataWidth) begin : piso

    // https://github.com/bespoke-silicon-group/basejump_stl/blob/a43571d2/bsg_dataflow/bsg_parallel_in_serial_out.sv
    bsg_parallel_in_serial_out #(
        .width_p(OutDataWidth),
        .els_p(InDataWidth / OutDataWidth),
        .hi_to_lo_p(0)
    ) piso (
        .clk_i,
        .reset_i(!rst_ni),

        .ready_and_o(in_ready_o),
        .valid_i(in_valid_i),
        .data_i(in_data_i),

        .yumi_i(out_ready_i && out_valid_o),
        .valid_o(out_valid_o),
        .data_o(out_data_o)
    );

end else if (gcd(InDataWidth, OutDataWidth) == InDataWidth) begin : sipo

    // https://github.com/bespoke-silicon-group/basejump_stl/blob/a43571d2/bsg_dataflow/bsg_serial_in_parallel_out_full.sv
    bsg_serial_in_parallel_out_full #(
        .width_p(InDataWidth),
        .els_p(OutDataWidth / InDataWidth),
        .hi_to_lo_p(0)
    ) sipo (
        .clk_i,
        .reset_i(!rst_ni),

        .ready_and_o(in_ready_o),
        .v_i(in_valid_i),
        .data_i(in_data_i),

        .yumi_i(out_ready_i && out_valid_o),
        .v_o(out_valid_o),
        .data_o(out_data_o)
    );

end else begin : non_multiple

    localparam int UnitWidth            = gcd(InDataWidth, OutDataWidth);
    localparam int UnitsPerInputWord    = InDataWidth/UnitWidth;
    localparam int UnitsPerOutputWord   = OutDataWidth/UnitWidth;
    localparam int StagingCapacityUnits = UnitsPerInputWord + UnitsPerOutputWord;
    localparam int StagingWidth         = StagingCapacityUnits*UnitWidth;
    localparam int StagingSizeWidth     = `SAFE_CLOG2(StagingCapacityUnits+1);

    logic [StagingWidth-1:0]     staging_data_d, staging_data_q;
    logic [StagingSizeWidth-1:0] staging_size_units_d, staging_size_units_q;

    assign out_valid_o = (staging_size_units_q >= UnitsPerOutputWord);
    assign out_data_o  = staging_data_q[OutDataWidth-1:0];
    assign in_ready_o  = staging_size_units_q <= (StagingCapacityUnits-UnitsPerInputWord);

    always_comb begin
        automatic logic [StagingSizeWidth-1:0] staging_size_units_after_output = staging_size_units_q;

        staging_data_d       = staging_data_q;
        staging_size_units_d = staging_size_units_q;

        if (out_valid_o && out_ready_i) begin
            staging_data_d                  = staging_data_q >> OutDataWidth;
            staging_size_units_after_output = (staging_size_units_q - UnitsPerOutputWord);
        end

        staging_size_units_d = staging_size_units_after_output;

        if (in_valid_i && in_ready_o) begin
            staging_data_d |= StagingWidth'(in_data_i) << (UnitWidth*staging_size_units_after_output);
            staging_size_units_d = (staging_size_units_after_output + UnitsPerInputWord);
        end
    end

    always_ff @(posedge clk_i) begin
        if (!rst_ni) begin
            staging_data_q       <= '0;
            staging_size_units_q <= '0;
        end else begin
            staging_data_q       <= staging_data_d;
            staging_size_units_q <= staging_size_units_d;
        end
    end

end

endmodule

`undef SAFE_CLOG2
