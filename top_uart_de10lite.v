module top_uart_de10lite #(
    parameter CLK_FREQ  = 50000000,
    parameter BAUD_RATE = 9600
)(
    input wire CLOCK_50,
    input wire RESET_N,
    input wire UART_RX,
    output wire UART_TX
);

    wire reset;
    assign reset = ~RESET_N;
    
    wire [7:0] rx_data;
    wire       rx_done;  
    
    reg [7:0] tx_data;
    reg       tx_start;
    wire      tx_busy;  
    
    // Codificación de estados 
    localparam [2:0] S_IDLE        = 3'b000, // Espera byte de inicio
                     S_RECEIVE     = 3'b001, // Recibe y acumula bytes
                     S_WAIT_MATH   = 3'b010, // Espera calculos de n^(5/2)   
                     S_SEND_HEADER = 3'b011, // Envia delimitador 0xAA
                     S_PROCESS     = 3'b100, // Carga y transmite cada byte de datos
                     S_WAIT_TX     = 3'b101, // Espera transmision de cada byte
                     S_SEND_FOOTER = 3'b110, // Envia delimitador 0x55
                     S_WAIT_FOOTER = 3'b111; // Espera fin de trama y regresa a IDLE
                     
    reg [2:0]   state;  
    reg [31:0]  complete_data; // 32 bits de entrada        
    reg [9:0]   index; 
    
    reg         pn_start;
    wire        pn_done;
    wire [95:0] pn_result;     // 96 bits (12 bytes en punto fijo Q80.16)
    
    // Módulo UART RX con parámetros dinámicos
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
    
    // Módulo UART TX con parámetros dinámicos
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
    
    // Coprocesador matemático n^(5/2)
    process_number PN (
        .data_in  (complete_data),  
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
            state         <= S_IDLE;
        end else begin 
            tx_start <= 1'b0;
            pn_start <= 1'b0;
        
            case(state) 
                
                S_IDLE: begin
                    index <= 10'd0;
                    if (rx_done && (rx_data == 8'hAA)) begin
                        complete_data <= 32'd0;
                        state         <= S_RECEIVE;   
                    end
                end
                
                S_RECEIVE: begin 
				    if (rx_done) begin
				        if (index < 10'd32) begin
				            // Los primeros 4 bytes recibidos siempre se guardan como datos,
				            // sin importar si su valor numerico es 0x55, 0xAA o cualquier otro.
				            complete_data[index +: 8] <= rx_data;  
				            index <= index + 10'd8; 
				        end else if (rx_data == 8'h55) begin
				            // Una vez completados los 4 bytes, se valida el delimitador final
				            pn_start <= 1'b1;
				            state    <= S_WAIT_MATH;    
				            index    <= 10'd0;
				        end else begin
				            // Si el 5to byte no es 0x55, la trama fue invalida
				            state    <= S_IDLE;
				        end
				    end
				end 
                
                S_WAIT_MATH: begin 
                    if (pn_done == 1'b1) begin  
                        index <= 10'd0;
                        state <= S_SEND_HEADER;  
                    end
                end 
                
                S_SEND_HEADER: begin    
                    if (!tx_busy) begin
                        tx_data  <= 8'hAA;
                        tx_start <= 1'b1; 
                        state    <= S_PROCESS; // Espera fin de 0xAA en S_PROCESS
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
                        if (index < 10'd88) begin // 12 bytes: index = 0, 8, 16, ..., 88
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
                        state <= S_IDLE; // Trama completa enviada
                end
                
                default: state <= S_IDLE;
                
            endcase 
        end
    end

endmodule
