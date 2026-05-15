
`define SAFE_CLOG2(x) ( (((x)==1) || ((x)==0)) ? 1 : $clog2(x) )

module ternip_dual_port_mem #(
    parameter int DATA_WIDTH      = 8,
    parameter int NUM_ENTRIES     = 256,
    parameter bit UNCOUPLED_READY = 0
) (
    input  logic                                clk_i,
    input  logic                                rst_ni,

    output logic                                a_request_ready_o,
    input  logic                                a_request_valid_i,
    input  logic                                a_request_write_not_read_i,
    input  logic [`SAFE_CLOG2(NUM_ENTRIES)-1:0] a_request_addr_i,
    input  logic [DATA_WIDTH-1:0]               a_request_w_data_i,

    input  logic                                a_read_ready_i,
    output logic                                a_read_valid_o,
    output logic [`SAFE_CLOG2(NUM_ENTRIES)-1:0] a_read_addr_o,
    output logic [DATA_WIDTH-1:0]               a_read_data_o,

    output logic                                b_request_ready_o,
    input  logic                                b_request_valid_i,
    input  logic                                b_request_write_not_read_i,
    input  logic [`SAFE_CLOG2(NUM_ENTRIES)-1:0] b_request_addr_i,
    input  logic [DATA_WIDTH-1:0]               b_request_w_data_i,

    input  logic                                b_read_ready_i,
    output logic                                b_read_valid_o,
    output logic [`SAFE_CLOG2(NUM_ENTRIES)-1:0] b_read_addr_o,
    output logic [DATA_WIDTH-1:0]               b_read_data_o
);

localparam int ADDR_WIDTH = `SAFE_CLOG2(NUM_ENTRIES);

logic [DATA_WIDTH-1:0] MEM [NUM_ENTRIES];

// port A
logic a_read_valid_d, a_read_valid_q1, a_read_valid_q2;
logic a_write_valid_d, a_write_valid_q1, a_write_valid_q2;
logic [ADDR_WIDTH-1:0] a_request_addr_q1, a_request_addr_q2;
logic [DATA_WIDTH-1:0] a_request_w_data_q1;
logic [DATA_WIDTH-1:0] a_read_data_q2;

logic                                             a_buffer_in_ready;
logic                                             a_buffer_in_valid;
logic [$bits({a_read_addr_o, a_read_data_o})-1:0] a_buffer_in_data;
logic                                             a_buffer_out_ready;
logic                                             a_buffer_out_valid;
logic [$bits({a_read_addr_o, a_read_data_o})-1:0] a_buffer_out_data;

assign a_read_valid_d  = a_request_valid_i && !a_request_write_not_read_i;
assign a_write_valid_d = a_request_valid_i &&  a_request_write_not_read_i;

logic a_stall1, a_stall2, a_stall3;
if (UNCOUPLED_READY) begin
    assign a_stall2 = !a_buffer_in_ready && a_read_valid_q2;
end else begin
    assign a_stall3 = !a_read_ready_i && a_read_valid_o;
    assign a_stall2 = a_stall3 && (a_read_valid_q2 || a_write_valid_q2);
end
assign a_stall1 = a_stall2 && (a_read_valid_q1 || a_write_valid_q1);
assign a_request_ready_o = !a_stall1;

always_ff @(posedge clk_i) begin
    if (!rst_ni) begin
        a_read_valid_q1  <= 0;
        a_write_valid_q1 <= 0;
    end else if (!a_stall1) begin
        a_read_valid_q1  <= a_read_valid_d;
        a_write_valid_q1 <= a_write_valid_d;
    end
end
always_ff @(posedge clk_i) begin
    if (!a_stall1) begin
        a_request_addr_q1   <= a_request_addr_i;
        a_request_w_data_q1 <= a_request_w_data_i;
    end
    `ifndef SYNTHESIS
    if (!rst_ni) begin
        a_request_addr_q1   <= 'x;
        a_request_w_data_q1 <= 'x;
    end
    `endif
end
always_ff @(posedge clk_i) begin
    if (!rst_ni) begin
        a_read_valid_q2  <= 0;
        a_write_valid_q2 <= 0;
    end else if (!a_stall2) begin
        a_read_valid_q2  <= a_read_valid_q1;
        a_write_valid_q2 <= a_write_valid_q1;
    end
end
always_ff @(posedge clk_i) begin
    if (!a_stall2) begin
        a_request_addr_q2 <= a_request_addr_q1;
    end
    `ifndef SYNTHESIS
    if (!rst_ni) begin
        a_request_addr_q2 <= 'x;
    end
    `endif
end

// port B
logic b_read_valid_d, b_read_valid_q1, b_read_valid_q2;
logic b_write_valid_d, b_write_valid_q1, b_write_valid_q2;
logic [ADDR_WIDTH-1:0] b_request_addr_q1, b_request_addr_q2;
logic [DATA_WIDTH-1:0] b_request_w_data_q1;
logic [DATA_WIDTH-1:0] b_read_data_q2;

logic                                             b_buffer_in_ready;
logic                                             b_buffer_in_valid;
logic [$bits({b_read_addr_o, b_read_data_o})-1:0] b_buffer_in_data;
logic                                             b_buffer_out_ready;
logic                                             b_buffer_out_valid;
logic [$bits({b_read_addr_o, b_read_data_o})-1:0] b_buffer_out_data;

assign b_read_valid_d  = b_request_valid_i && !b_request_write_not_read_i;
assign b_write_valid_d = b_request_valid_i &&  b_request_write_not_read_i;

logic b_stall1, b_stall2, b_stall3;
if (UNCOUPLED_READY) begin
    assign b_stall2 = !b_buffer_in_ready && b_read_valid_q2;
end else begin
    assign b_stall3 = !b_read_ready_i && b_read_valid_o;
    assign b_stall2 = b_stall3 && (b_read_valid_q2 || b_write_valid_q2);
end
assign b_stall1 = b_stall2 && (b_read_valid_q1 || b_write_valid_q1);
assign b_request_ready_o = !b_stall1;

always_ff @(posedge clk_i) begin
    if (!rst_ni) begin
        b_read_valid_q1  <= 0;
        b_write_valid_q1 <= 0;
    end else if (!b_stall1) begin
        b_read_valid_q1  <= b_read_valid_d;
        b_write_valid_q1 <= b_write_valid_d;
    end
end
always_ff @(posedge clk_i) begin
    if (!b_stall1) begin
        b_request_addr_q1   <= b_request_addr_i;
        b_request_w_data_q1 <= b_request_w_data_i;
    end
    `ifndef SYNTHESIS
    if (!rst_ni) begin
        b_request_addr_q1   <= 'x;
        b_request_w_data_q1 <= 'x;
    end
    `endif
end
always_ff @(posedge clk_i) begin
    if (!rst_ni) begin
        b_read_valid_q2  <= 0;
        b_write_valid_q2 <= 0;
    end else if (!b_stall2) begin
        b_read_valid_q2  <= b_read_valid_q1;
        b_write_valid_q2 <= b_write_valid_q1;
    end
end
always_ff @(posedge clk_i) begin
    if (!b_stall2) begin
        b_request_addr_q2 <= b_request_addr_q1;
    end
    `ifndef SYNTHESIS
    if (!rst_ni) begin
        b_request_addr_q2 <= 'x;
    end
    `endif
end

// shared MEM
always_ff @(posedge clk_i) begin
    if (!a_stall2) begin
        if (a_write_valid_q1) begin
            MEM[a_request_addr_q1] <= a_request_w_data_q1;
        end else if (a_read_valid_q1) begin
            a_read_data_q2 <= MEM[a_request_addr_q1];
        end
    end
end
always_ff @(posedge clk_i) begin
    if (!b_stall2) begin
        if (b_write_valid_q1) begin
            MEM[b_request_addr_q1] <= b_request_w_data_q1;
        end else if (b_read_valid_q1) begin
            b_read_data_q2 <= MEM[b_request_addr_q1];
        end
    end
end

// output buffer A
if (UNCOUPLED_READY) begin : a_uncoupled
    assign a_buffer_in_valid = a_read_valid_q2;
    assign a_buffer_in_data  = {a_request_addr_q2, a_read_data_q2};

    assign a_buffer_out_ready = a_read_ready_i;
    assign a_read_valid_o = a_buffer_out_valid;
    assign {a_read_addr_o, a_read_data_o} = a_buffer_out_data;

    ternip_pipelined_interconnect #(
        .DataWidth($bits(a_buffer_in_data)),
        .NumStages(1)
    ) a_buffer (
        .clk_i,
        .rst_ni,
        .in_ready_o(a_buffer_in_ready),
        .in_valid_i(a_buffer_in_valid),
        .in_data_i (a_buffer_in_data),
        .out_ready_i(a_buffer_out_ready),
        .out_valid_o(a_buffer_out_valid),
        .out_data_o (a_buffer_out_data)
    );
end else begin : a_coupled
    always_ff @(posedge clk_i) begin
        if (!rst_ni) begin
            a_read_valid_o <= 1'b0;
        end else if (!a_stall3) begin
            a_read_valid_o <= a_read_valid_q2;
        end
    end
    always_ff @(posedge clk_i) begin
        if (!a_stall3) begin
            a_read_addr_o <= a_read_valid_q2 ? a_request_addr_q2 : 'x;
            a_read_data_o <= a_read_valid_q2 ? a_read_data_q2    : 'x;
        end
        `ifndef SYNTHESIS
        if (!rst_ni) begin
            a_read_addr_o <= 'x;
            a_read_data_o <= 'x;
        end
        `endif
    end
end

// output buffer B
if (UNCOUPLED_READY) begin : b_uncoupled
    assign b_buffer_in_valid = b_read_valid_q2;
    assign b_buffer_in_data  = {b_request_addr_q2, b_read_data_q2};

    assign b_buffer_out_ready = b_read_ready_i;
    assign b_read_valid_o = b_buffer_out_valid;
    assign {b_read_addr_o, b_read_data_o} = b_buffer_out_data;

    ternip_pipelined_interconnect #(
        .DataWidth($bits(b_buffer_in_data)),
        .NumStages(1)
    ) b_buffer (
        .clk_i,
        .rst_ni,
        .in_ready_o(b_buffer_in_ready),
        .in_valid_i(b_buffer_in_valid),
        .in_data_i (b_buffer_in_data),
        .out_ready_i(b_buffer_out_ready),
        .out_valid_o(b_buffer_out_valid),
        .out_data_o (b_buffer_out_data)
    );
end else begin : b_coupled
    always_ff @(posedge clk_i) begin
        if (!rst_ni) begin
            b_read_valid_o <= 1'b0;
        end else if (!b_stall3) begin
            b_read_valid_o <= b_read_valid_q2;
        end
    end
    always_ff @(posedge clk_i) begin
        if (!b_stall3) begin
            b_read_addr_o <= b_read_valid_q2 ? b_request_addr_q2 : 'x;
            b_read_data_o <= b_read_valid_q2 ? b_read_data_q2    : 'x;
        end
        `ifndef SYNTHESIS
        if (!rst_ni) begin
            b_read_addr_o <= 'x;
            b_read_data_o <= 'x;
        end
        `endif
    end
end

endmodule

`undef SAFE_CLOG2
