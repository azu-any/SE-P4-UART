module uart_tx #(
    parameter CLK_FREQ  = 50000000,
    parameter BAUD_RATE = 9600
)(
    input  wire       clk,
    input  wire       reset,
    input  wire       tx_start,
    input  wire [7:0] data_in,
    output reg        tx,
    output reg        busy
);

localparam integer CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;

reg [15:0] clk_count;
reg [3:0]  bit_index;
reg [9:0]  tx_shift;

always @(posedge clk or posedge reset) begin
    if (reset) begin
        tx        <= 1'b1;
        busy      <= 1'b0;
        clk_count <= 0;
        bit_index <= 0;
        tx_shift  <= 10'b1111111111;
    end else begin
        if (!busy) begin
            tx <= 1'b1;
            if (tx_start) begin
                busy      <= 1'b1;
                tx_shift  <= {1'b1, data_in, 1'b0};
                clk_count <= 0;
                bit_index <= 0;
            end
        end else begin
            tx <= tx_shift[0];
            if (clk_count < CLKS_PER_BIT - 1) begin
                clk_count <= clk_count + 1;
            end else begin
                clk_count <= 0;
                tx_shift  <= {1'b1, tx_shift[9:1]};
                if (bit_index < 9)
                    bit_index <= bit_index + 1;
                else begin
                    busy <= 1'b0;
                    tx   <= 1'b1;
                end
            end
        end
    end
end

endmodule