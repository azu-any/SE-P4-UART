module square_root (
    input  wire [31:0] data_in,   // Entrada entera n (32 bits)
    input  wire        clk,
    input  wire        reset,
    input  wire        start,
    output reg  [31:0] sqrt_out,  // Salida Q16.16: [31:16] Entero, [15:0] Fraccionario
    output reg         done
);

    localparam [1:0] S_IDLE = 2'b00,
                     S_CALC = 2'b01,
                     S_DONE = 2'b10;

    reg [1:0]  state;
    reg [4:0]  iter;     // Cuenta de 31 hacia 0 (32 iteraciones en total)
    reg [63:0] rem_data; // Entrada extendida: 32 bits enteros + 32 ceros a la derecha
    reg [33:0] r;        // Residuo acumulado (34 bits para desplazamiento y prueba)
    reg [31:0] q;        // Raíz acumulada (32 bits en Q16.16)

    wire [33:0] r_next;
    wire [33:0] test_val;

    // 1. Desplazar el residuo 2 bits a la izquierda e incorporar el siguiente par de bits
    assign r_next   = {r[31:0], rem_data[63:62]};

    // 2. Valor de prueba: {q, 2'b01} (32 bits + 2 bits = 34 bits)
    assign test_val = {q, 2'b01};

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state    <= S_IDLE;
            iter     <= 5'd0;
            rem_data <= 64'd0;
            r        <= 34'd0;
            q        <= 32'd0;
            sqrt_out <= 32'd0;
            done     <= 1'b0;
        end else begin
            done <= 1'b0; // Pulso por defecto

            case (state)
                S_IDLE: begin
                    if (start) begin
                        // Se colocan 32 ceros a la derecha para obtener 16 bits de fracción
                        rem_data <= {data_in, 32'd0};
                        r        <= 34'd0;
                        q        <= 32'd0;
                        iter     <= 5'd31; // 32 iteraciones (de 31 hacia 0)
                        state    <= S_CALC;
                    end
                end

                S_CALC: begin
                    // Evaluar sustracción
                    if (r_next >= test_val) begin
                        r <= r_next - test_val;
                        q <= {q[30:0], 1'b1};
                    end else begin
                        r <= r_next;
                        q <= {q[30:0], 1'b0};
                    end

                    // Desplazar 2 bits a la izquierda para el siguiente par
                    rem_data <= {rem_data[61:0], 2'b00};

                    if (iter == 5'd0) begin
                        state <= S_DONE;
                    end else begin
                        iter  <= iter - 5'd1;
                    end
                end

                S_DONE: begin
                    sqrt_out <= q;     // [31:16] = Parte entera, [15:0] = Parte decimal
                    done     <= 1'b1;
                    state    <= S_IDLE;
                end

                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end

endmodule