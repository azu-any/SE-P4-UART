`timescale 1ns / 1ps

module top_uart_de10lite #(
    parameter CLK_FREQ  = 50000000,
    parameter BAUD_RATE = 9600
)(
    input  wire CLOCK_50,
    input  wire RESET_N,
    input  wire UART_RX,
    output wire UART_TX
);

    // Reset activo en alto para la lógica interna
    wire reset;
    assign reset = ~RESET_N;
    
    // Señales de la interfaz UART RX
    wire [7:0] rx_data;
    wire       rx_done;  
    
    // Señales de la interfaz UART TX
    reg  [7:0] tx_data;
    reg        tx_start;
    wire       tx_busy;  
    
    // Codificación de estados de la FSM
    localparam [2:0] S_IDLE        = 3'b000, // Espera delimitador 0xAA
                     S_RECEIVE     = 3'b001, // Recibe 4 bytes de datos y valida 0x55
                     S_WAIT_MATH   = 3'b010, // Espera que finalice el coprocesador n^(5/2)
                     S_SEND_HEADER = 3'b011, // Carga e inicia envio de cabecera 0xAA
                     S_WAIT_HEADER = 3'b100, // Espera a que 0xAA salga completamente
                     S_PROCESS     = 3'b101, // Carga y transmite cada byte del resultado
                     S_WAIT_TX     = 3'b110, // Espera salida de cada byte y avanza indice
                     S_SEND_FOOTER = 3'b111; // Envia delimitador final 0x55 y regresa a IDLE
                     
    reg [2:0]  state;  
    reg [31:0] complete_data; // 32 bits para n
    reg [9:0]  index; 
    
    // Conexiones hacia el bloque matemático
    reg        pn_start;
    wire       pn_done;
    wire [95:0] pn_result;    // 96 bits en formato Q80.16 (12 bytes)
    
    // -------------------------------------------------------------------------
    // Instanciación del Receptor UART
    // -------------------------------------------------------------------------
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
    
    // -------------------------------------------------------------------------
    // Instanciación del Transmisor UART
    // -------------------------------------------------------------------------
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
    
    // -------------------------------------------------------------------------
    // Coprocesador Matemático n^(5/2)
    // -------------------------------------------------------------------------
    process_number PN (
        .data_in  (complete_data),  
        .clk      (CLOCK_50),
        .reset    (reset),
        .start    (pn_start),
        .data_out (pn_result),
        .done     (pn_done)
    );  
    
    // -------------------------------------------------------------------------
    // Máquina de Estados Principal
    // -------------------------------------------------------------------------
    always @(posedge CLOCK_50 or posedge reset) begin
        if (reset) begin
            tx_data       <= 8'd0;
            tx_start      <= 1'b0;
            pn_start      <= 1'b0;
            index         <= 10'd0;
            complete_data <= 32'd0;
            state         <= S_IDLE;
        end else begin 
            // Pulsos por defecto
            tx_start <= 1'b0;
            pn_start <= 1'b0;
        
            case (state) 
                
                // 1. Espera delimitador inicial 0xAA
                S_IDLE: begin
                    index <= 10'd0;
                    if (rx_done && (rx_data == 8'hAA)) begin
                        complete_data <= 32'd0;
                        state         <= S_RECEIVE;   
                    end
                end
                
                // 2. Recepción blindada de 4 bytes antes de aceptar delimitador 0x55
                S_RECEIVE: begin 
                    if (rx_done) begin
                        if (index < 10'd32) begin
                            // Almacenar bytes 0, 1, 2 y 3 (Little-Endian)
                            complete_data[index +: 8] <= rx_data;  
                            index <= index + 10'd8; 
                        end else if (rx_data == 8'h55) begin
                            // Validar delimitador final únicamente tras recibir los 4 bytes
                            pn_start <= 1'b1;
                            state    <= S_WAIT_MATH;    
                            index    <= 10'd0;
                        end else begin
                            // Trama corrupta o fuera de sincronía
                            state    <= S_IDLE;
                        end 
                    end
                end 
                
                // 3. Espera a que el bloque matemático termine su cálculo
                S_WAIT_MATH: begin 
                    if (pn_done) begin  
                        index <= 10'd0;
                        state <= S_SEND_HEADER;  
                    end
                end 
                
                // 4. Iniciar transmisión de cabecera 0xAA
                S_SEND_HEADER: begin    
                    if (!tx_busy) begin
                        tx_data  <= 8'hAA;
                        tx_start <= 1'b1; 
                        state    <= S_WAIT_HEADER; 
                    end 
                end 
                
                // 5. Esperar que 0xAA salga completamente del transmisor
                S_WAIT_HEADER: begin
                    if (!tx_busy && !tx_start) begin
                        index <= 10'd0;
                        state <= S_PROCESS;
                    end
                end
                
                // 6. Cargar y disparar cada byte del resultado de 96 bits
                S_PROCESS: begin  
                    if (!tx_busy && !tx_start) begin
                        tx_data  <= pn_result[index +: 8];
                        tx_start <= 1'b1; 
                        state    <= S_WAIT_
