module uart_rx #(
    parameter CLK_FREQ  = 50000000,
    parameter BAUD_RATE = 9600
)(
    input  wire       clk,
    input  wire       reset,
    input  wire       rx,
    output reg  [7:0] data_out,
    output reg        data_valid
);

	localparam integer CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;
	localparam integer HALF_BIT     = CLKS_PER_BIT / 2;	  
	
	
	// Codificación de estados 
	// Mayor control sobre el receptor
	localparam [1:0] S_IDLE      = 2'b00, // Espera informacion
		             S_START_BIT = 2'b01, // Confirma que el bit de inicio sea recibido
		             S_DATA_BITS = 2'b10, // Recibe datos
		             S_STOP_BIT  = 2'b11; // Indica que todos los datos se recibieron
	
	reg [1:0]  state;
	reg [15:0] clk_count;
	reg [3:0]  bit_index;
	reg [7:0]  rx_shift;
	
	always @(posedge clk or posedge reset) begin
	    if (reset) begin 
			state <= S_IDLE;  // Modo espera
	        clk_count  <= 0;
	        bit_index  <= 0;
	        rx_shift   <= 0;
	        data_out   <= 8'd0;
	        data_valid <= 1'b0;	  
			
			
	    end else begin	  
			// Pulso data_valid por defecto en bajo
	        data_valid <= 1'b0;
			
			case(state)	
				
				// REPOSO
	        	S_IDLE: begin
					clk_count <= 0;
	                bit_index <= 0;	
					
	            	if (rx == 1'b0)	
						state <= S_START_BIT; // Flanco de bajada detectado en t = 0.0
				end
				
				// VALIDACION BIT INICIAL
				S_START_BIT: begin
					if (clk_count < HALF_BIT - 1)
						clk_count <= clk_count + 1;
					else begin
	                	clk_count <= 0; // Reiniciar contador
						if (rx == 1'b0) 
							state <= S_DATA_BITS; // Start bit confirmado t = 0.5
						else 
							state <= S_IDLE;
					end
				end
				
				// RECEPCION DE DATOS
				S_DATA_BITS: begin		
					if (clk_count < CLKS_PER_BIT - 1)
						clk_count <= clk_count + 1;
					else begin
						clk_count <= 0;
	                    rx_shift[bit_index] <= rx;
						
						if (bit_index < 7)
	                    	bit_index <= bit_index + 1;	   
						else begin 
							bit_index <= 0;
							state <= S_STOP_BIT;
						end
					end
				end
				
				// BIT DE PARO Y SALIDA
				S_STOP_BIT: begin
					if (clk_count < CLKS_PER_BIT - 1)
						clk_count <= clk_count + 1;
					else begin										
						clk_count <= 0;
	                    data_out   <= rx_shift;
	                    data_valid <= 1'b1;	 
						state <= S_IDLE;
	                end
	            end	 
				
				default: state <= S_IDLE;
	        endcase
	    end
	end

endmodule
