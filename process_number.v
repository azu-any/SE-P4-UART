module process_number (
    input  wire [31:0] data_in,   // Base entera n (32 bits)
    input  wire [7:0]  p_in,      // Exponente p (8 bits) para calcular n^(p/2)
    input  wire        clk,
    input  wire        reset,
    input  wire        start,
    output reg  [95:0] data_out,  // Resultado n^(p/2) en formato Q80.16
    output reg         done
);

    localparam [1:0] S_IDLE = 2'b00,
                     S_CALC = 2'b01, // Calcula n^(p >> 1) y sqrt(n) en paralelo
                     S_MULT = 2'b10, // Combina parte entera con sqrt(n) si p es impar
                     S_DONE = 2'b11;

    reg [1:0]  state;
    reg [79:0] pow_acc;      // Acumulador de 80 bits para n^(p/2 entero)
    reg [6:0]  exp_cnt;      // Contador de multiplicaciones restantes (p >> 1)
    reg        p_is_odd;     // Guarda p_in[0] (1 si p es impar, 0 si p es par)

    reg         sqrt_start;
    wire [31:0] sqrt_res;    // Raíz cuadrada en Q16.16
    wire        sqrt_done;
    reg         sqrt_ready;  // Bandera que recuerda si sqrt_root ya terminó

    // Instanciación del módulo square_root (sin cambios)
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
            pow_acc    <= 80'd0;
            exp_cnt    <= 7'd0;
            p_is_odd   <= 1'b0;
            sqrt_start <= 1'b0;
            sqrt_ready <= 1'b0;
            done       <= 1'b0;
        end else begin
            done       <= 1'b0;
            sqrt_start <= 1'b0;

            if (sqrt_done)
                sqrt_ready <= 1'b1;

            case (state)
                S_IDLE: begin
                    if (start) begin
                        pow_acc    <= 80'd1;         // Inicializar acumulador en 1 (n^0 = 1)
                        exp_cnt    <= p_in[7:1];     // Cantidad de multiplicaciones = floor(p / 2)
                        p_is_odd   <= p_in[0];       // 1 si necesitamos multiplicar por sqrt(n)
                        sqrt_start <= 1'b1;          // Iniciar cálculo de raíz en paralelo
                        sqrt_ready <= 1'b0;
                        state      <= S_CALC;
                    end
                end

                S_CALC: begin
                    // Multiplicar iterativamente pow_acc * n hasta que exp_cnt llegue a 0
                    if (exp_cnt > 7'd0) begin
                        pow_acc <= pow_acc * data_in;
                        exp_cnt <= exp_cnt - 7'd1;
                    end

                    // Avanzar cuando AMBOS (la potencia entera y la raíz cuadrada) hayan terminado
                    if ((exp_cnt == 7'd0) && (sqrt_ready || sqrt_done)) begin
                        state <= S_MULT;
                    end
                end

                S_MULT: begin
                    if (data_in == 32'd0 && (p_in != 8'd0)) begin
                        data_out <= 96'd0;                 // 0 elevado a cualquier p > 0 es 0
                    end else if (p_is_odd) begin
                        // Si p es impar: (Entero 80b) * (Q16.16 32b) -> truncado a Q80.16 (96b)
                        data_out <= pow_acc * sqrt_res;
                    end else begin
                        // Si p es par: desplazar 16 bits a la izquierda para formato Q80.16
                        data_out <= {pow_acc, 16'd0};
                    end
                    state <= S_DONE;
                end

                S_DONE: begin
                    done  <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
