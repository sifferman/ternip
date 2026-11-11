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

function automatic int min(int a, int b);
    return (a < b) ? a : b;
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
    localparam int StagingCapacityUnits = UnitsPerInputWord + UnitsPerOutputWord
                                        + min(UnitsPerInputWord, UnitsPerOutputWord) - 1;
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

`ifdef FORMAL

    localparam int FvStreamPositionWidth = StagingSizeWidth + 1;
    typedef logic [FvStreamPositionWidth-1:0] fv_stream_position_t;

    function automatic logic [UnitWidth-1:0] fv_generate_test_pattern(fv_stream_position_t stream_position);
        localparam int Repeats = UnitWidth / FvStreamPositionWidth + 1;
        logic [Repeats*FvStreamPositionWidth-1:0] repeated_position = {Repeats{stream_position}};
        return repeated_position[UnitWidth-1:0];
    endfunction

    fv_stream_position_t              fv_input_stream_position_q;
    fv_stream_position_t              fv_output_stream_position_q;
    logic [FvStreamPositionWidth-1:0] fv_num_units_still_inside_d;
    logic [FvStreamPositionWidth-1:0] fv_num_units_still_inside_q;
    logic                             fv_has_started_flowing_q;
    logic                             fv_never_stalled_by_environment_q;
    logic [InDataWidth-1:0]           fv_expected_in_data;
    logic [OutDataWidth-1:0]          fv_expected_out_data;

    always_comb begin
        for (int i = 0; i < UnitsPerInputWord;  i++)
            fv_expected_in_data [i*UnitWidth +: UnitWidth] = fv_generate_test_pattern(fv_input_stream_position_q  + i);
        for (int i = 0; i < UnitsPerOutputWord; i++)
            fv_expected_out_data[i*UnitWidth +: UnitWidth] = fv_generate_test_pattern(fv_output_stream_position_q + i);
    end

    always_comb begin
        fv_num_units_still_inside_d = fv_num_units_still_inside_q;
        if (in_valid_i  && in_ready_o)  fv_num_units_still_inside_d += UnitsPerInputWord;
        if (out_valid_o && out_ready_i) fv_num_units_still_inside_d -= UnitsPerOutputWord;
    end

    wire fv_complete_word_inside = fv_num_units_still_inside_q >= UnitsPerOutputWord;
    wire fv_smaller_port_ready   = (UnitsPerInputWord < UnitsPerOutputWord) ? in_ready_o : out_valid_o;

    always_ff @(posedge clk_i) begin
        assume (in_data_i == fv_expected_in_data);

        if (!rst_ni) begin
            fv_input_stream_position_q        <= '0;
            fv_output_stream_position_q       <= '0;
            fv_num_units_still_inside_q       <= '0;
            fv_has_started_flowing_q          <= 1'b0;
            fv_never_stalled_by_environment_q <= 1'b1;
        end else begin
            if (in_valid_i  && in_ready_o)  fv_input_stream_position_q  <= fv_input_stream_position_q  + UnitsPerInputWord;
            if (out_valid_o && out_ready_i) fv_output_stream_position_q <= fv_output_stream_position_q + UnitsPerOutputWord;
            if (fv_smaller_port_ready)      fv_has_started_flowing_q    <= 1'b1;

            fv_num_units_still_inside_q       <= fv_num_units_still_inside_d;
            fv_never_stalled_by_environment_q <= fv_never_stalled_by_environment_q && in_valid_i && out_ready_i;

            no_lost_or_reordered_data: assert (!(out_valid_o && out_ready_i) || out_data_o == fv_expected_out_data);
            out_valid_when_word_ready: assert (!fv_complete_word_inside || out_valid_o);
            no_deadlock:               assert (in_ready_o || out_valid_o);
            no_wasted_cycles:          assert (!fv_never_stalled_by_environment_q || !fv_has_started_flowing_q || fv_smaller_port_ready);

            fv_data_moves: cover (out_valid_o && out_ready_i);
        end
    end

`endif

end

endmodule

`undef SAFE_CLOG2
