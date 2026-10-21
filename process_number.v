module process_number (
    input  wire [31:0]  data_in,   // Entrada n de 32 bits
    input  wire         clk,
    input  wire         reset,
    input  wire         start,
    output reg  [95:0]  data_out,  // n^(5/2) en formato Q80.16 (16 bits fraccionarios)
    output reg          done
);				

    // Codificación de estados
    localparam [1:0] S_IDLE      = 2'b00,
                     S_WAIT_SQRT = 2'b01,
                     S_MULT      = 2'b10,
                     S_DONE      = 2'b11;

    reg [1:0]   state; 
    reg [63:0]  n2_reg;      // 32 x 32 bits = 64 bits
    reg         sqrt_start; 
    wire [31:0] sqrt_res;    // Raíz cuadrada de 32 bits en Q16.16
    wire        sqrt_done;  

    // Instanciación del módulo square_root adaptado a 32 bits
    square_root u_sqrt (	
        .data_in  (data_in),
        .clk      (clk),
        .reset    (reset),	
        .start    (sqrt_start),
        .sqrt_out (sqrt_res),
        .done     (sqrt_done)
    );	
	
    always @(posedge clk or posedge reset) begin
        if (reset) begin 
            state      <= S_IDLE;
            data_out   <= 96'd0;
            n2_reg     <= 64'd0;
            sqrt_start <= 1'b0;
            done       <= 1'b0;
        end else begin
            done       <= 1'b0; // Pulso en bajo por defecto
            sqrt_start <= 1'b0; // sqrt_start dura solo 1 ciclo
		
            case (state)
                S_IDLE: begin 
                    if (start) begin 
                        n2_reg     <= data_in * data_in; // n^2 (entero de 64 bits)
                        sqrt_start <= 1'b1;              // Disparar raíz
                        state      <= S_WAIT_SQRT; 
                    end
                end
				
                S_WAIT_SQRT: begin
                    if (sqrt_done) begin
                        state <= S_MULT;
                    end
                end
				
                S_MULT: begin		
                    // n^2 (entero) * sqrt (Q16.16) = resultado en Q80.16
                    data_out <= n2_reg * sqrt_res;	
                    state    <= S_DONE;
                end		  
				
                S_DONE: begin
                    done  <= 1'b1;
                    state <= S_IDLE;
                end
				
                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule