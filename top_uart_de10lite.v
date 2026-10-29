module top_uart_de10lite #(
    parameter CLK_FREQ  = 50000000,
    parameter BAUD_RATE = 9600
)(
    input  wire CLOCK_50,
    input  wire RESET_N,
    input  wire UART_RX,
    output wire UART_TX
);

    wire reset;
    assign reset = ~RESET_N;

    wire [7:0] rx_data;
    wire       rx_done;

    reg  [7:0] tx_data;
    reg        tx_start;
    wire       tx_busy;

    // Estados del controlador UART y del coprocesador matematico.
    localparam [2:0] S_IDLE        = 3'b000,
                     S_RECEIVE     = 3'b001,
                     S_WAIT_MATH   = 3'b010,
                     S_SEND_HEADER = 3'b011,
                     S_PROCESS     = 3'b100,
                     S_WAIT_TX     = 3'b101,
                     S_SEND_FOOTER = 3'b110,
                     S_WAIT_FOOTER = 3'b111;

    reg [2:0]  state;
    reg [31:0] complete_data; // n: 4 bytes recibidos en Little-Endian
    reg [7:0]  exponent_p;    // p: numerador del exponente p/2
    reg [9:0]  index;         // Indice en bits para ensamblar/transmitir bytes

    reg        pn_start;
    wire       pn_done;
    wire [95:0] pn_result;    // Resultado Q80.16, transmitido como 12 bytes

    // Receptor UART: un byte por pulso rx_done.
    uart_rx #(
        .CLK_FREQ(CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) RX0 (
        .clk(CLOCK_50),
        .reset(reset),
        .rx(UART_RX),
        .data_out(rx_data),
        .data_valid(rx_done)
    );

    // Transmisor UART hacia el ESP32.
    uart_tx #(
        .CLK_FREQ(CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) TX0 (
        .clk(CLOCK_50),
        .reset(reset),
        .tx_start(tx_start),
        .data_in(tx_data),
        .tx(UART_TX),
        .busy(tx_busy)
    );

    // Coprocesador configurable: calcula n^(p/2).
    process_number PN (
        .data_in  (complete_data),
        .p_in     (exponent_p),
        .clk      (CLOCK_50),
        .reset    (reset),
        .start    (pn_start),
        .data_out (pn_result),
        .done     (pn_done)
    );

    always @(posedge CLOCK_50 or posedge reset) begin
        if (reset) begin
            tx_data       <= 8'd0;
            tx_start      <= 1'b0;
            pn_start      <= 1'b0;
            index         <= 10'd0;
            complete_data <= 32'd0;
            exponent_p    <= 8'd0;
            state         <= S_IDLE;
        end else begin
            // Los pulsos de inicio duran un ciclo de reloj.
            tx_start <= 1'b0;
            pn_start <= 1'b0;

            case (state)
                S_IDLE: begin
                    index <= 10'd0;
                    if (rx_done && (rx_data == 8'hAA)) begin
                        complete_data <= 32'd0;
                        exponent_p    <= 8'd0;
                        state         <= S_RECEIVE;
                    end
                end

                S_RECEIVE: begin
                    if (rx_done) begin
                        if (index < 10'd32) begin
                            // Recibir los 4 bytes de n. Se permiten todos los
                            // valores, incluidos 0xAA y 0x55, dentro del payload.
                            complete_data[index +: 8] <= rx_data;
                            index <= index + 10'd8;
                        end else if (index == 10'd32) begin
                            // Quinto byte: numerador p del exponente p/2.
                            exponent_p <= rx_data;
                            index <= 10'd40;
                        end else begin
                            // Despues de n (4 bytes) y p (1 byte), debe venir 0x55.
                            if (rx_data == 8'h55) begin
                                pn_start <= 1'b1;
                                state    <= S_WAIT_MATH;
                                index    <= 10'd0;
                            end else begin
                                // Trama mal formada: volver a esperar cabecera.
                                state <= S_IDLE;
                                index <= 10'd0;
                            end
                        end
                    end
                end

                S_WAIT_MATH: begin
                    if (pn_done) begin
                        index <= 10'd0;
                        state <= S_SEND_HEADER;
                    end
                end

                S_SEND_HEADER: begin
                    if (!tx_busy) begin
                        tx_data  <= 8'hAA;
                        tx_start <= 1'b1;
                        state    <= S_PROCESS;
                    end
                end

                S_PROCESS: begin
                    if (!tx_busy && !tx_start) begin
                        tx_data  <= pn_result[index +: 8];
                        tx_start <= 1'b1;
                        state    <= S_WAIT_TX;
                    end
                end

                S_WAIT_TX: begin
                    if (!tx_busy && !tx_start) begin
                        if (index < 10'd88) begin
                            // 12 bytes de resultado: indices 0, 8, ..., 88.
                            index <= index + 10'd8;
                            state <= S_PROCESS;
                        end else begin
                            state <= S_SEND_FOOTER;
                        end
                    end
                end

                S_SEND_FOOTER: begin
                    if (!tx_busy && !tx_start) begin
                        tx_data  <= 8'h55;
                        tx_start <= 1'b1;
                        state    <= S_WAIT_FOOTER;
                    end
                end

                S_WAIT_FOOTER: begin
                    if (!tx_busy && !tx_start)
                        state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
