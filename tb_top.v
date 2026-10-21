`timescale 1ns / 1ps

module tb_top_uart_de10lite;

    // Parámetros de simulación
    // Se puede usar 2500000 para acelerar la simulación en ModelSim, o 9600 para prueba estándar
    localparam CLK_FREQ     = 50000000;
    localparam BAUD_RATE    = 9600;                       // 2.5 Mbaud para acelerar simulación
    localparam CLK_PERIOD   = 20;                            // 20 ns = 50 MHz
    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;          // 20 ciclos por bit
    localparam BIT_PERIOD   = CLKS_PER_BIT * CLK_PERIOD;     // 400 ns por bit

    // Señales DUT
    reg  CLOCK_50;
    reg  RESET_N;
    reg  UART_RX;
    wire UART_TX;

    // Registros de captura
    reg [511:0] rx_payload_captured;
    reg [7:0]   rx_header;
    reg [7:0]   rx_footer;

    // Variables de decodificación en punto flotante
    real real_result;
    real expected_result;
    real abs_error;

    // Instancia del módulo principal
    top_uart_de10lite #(
        .CLK_FREQ(CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) uut (
        .CLOCK_50(CLOCK_50),
        .RESET_N(RESET_N),
        .UART_RX(UART_RX),
        .UART_TX(UART_TX)
    );

    // 1. Generador de reloj de 50 MHz
    initial begin
        CLOCK_50 = 1'b0;
        forever #(CLK_PERIOD / 2) CLOCK_50 = ~CLOCK_50;
    end

    // 2. Tarea para enviar un solo byte por UART
    task send_uart_byte(input [7:0] data);
        integer bit_idx;
        begin
            UART_RX = 1'b0; // Start bit
            #(BIT_PERIOD);

            for (bit_idx = 0; bit_idx < 8; bit_idx = bit_idx + 1) begin
                UART_RX = data[bit_idx];
                #(BIT_PERIOD);
            end

            UART_RX = 1'b1; // Stop bit
            #(BIT_PERIOD);
        end
    endtask

    // 3. Tarea para enviar trama con n de 32 bits (4 bytes little-endian)
    task send_uart_frame_32bit(input [31:0] n_val);
        begin
            $display("\n[%0t ns] === Enviando trama UART: n = %0d ===", $time, n_val);

            // Delimitador de inicio
            send_uart_byte(8'hAA);

            // Carga útil de 4 bytes (LSB primero)
            send_uart_byte(n_val[7:0]);
            send_uart_byte(n_val[15:8]);
            send_uart_byte(n_val[23:16]);
            send_uart_byte(n_val[31:24]);

            // Delimitador de fin
            send_uart_byte(8'h55);

            $display("[%0t ns] === Trama enviada hacia FPGA ===", $time);
        end
    endtask

    // 4. Captura de un byte individual en UART_TX
    task capture_uart_byte(output [7:0] captured_byte);
        integer bit_idx;
        begin
            @(negedge UART_TX);
            #(BIT_PERIOD / 2); // Centro del Start bit

            for (bit_idx = 0; bit_idx < 8; bit_idx = bit_idx + 1) begin
                #(BIT_PERIOD);
                captured_byte[bit_idx] = UART_TX;
            end

            #(BIT_PERIOD);     // Stop bit
            #(BIT_PERIOD / 2);
        end
    endtask

    // 5. Captura de respuesta empaquetada (Header + 12 bytes / 96 bits + Footer)
    task capture_response(output [7:0] header, output [511:0] payload, output [7:0] footer);
        integer byte_idx;
        reg [7:0] temp_byte;
        begin
            capture_uart_byte(header);

            payload = 512'd0;
            // Se capturan los 12 bytes del resultado (96 bits en formato Q80.16)
            for (byte_idx = 0; byte_idx < 12; byte_idx = byte_idx + 1) begin
                capture_uart_byte(temp_byte);
                payload[byte_idx*8 +: 8] = temp_byte;
            end

            capture_uart_byte(footer);
        end
    endtask

    // 6. Tarea para verificar y reportar resultados en punto fijo Q.16
    task verify_result(input [31:0] n_val);
        integer int_part;
        integer frac_part;
        begin
            capture_response(rx_header, rx_payload_captured, rx_footer);

            // Separar parte entera y parte decimal (Q.16)
            int_part  = rx_payload_captured[63:16]; // Bits superiores a 16
            frac_part = rx_payload_captured[15:0];  // 16 bits fraccionarios

            // Conversión a valor real dividiendo entre 2^16 (65536.0)
            real_result     = rx_payload_captured[63:0] / 65536.0;
            expected_result = (n_val * n_val) * $sqrt(n_val);
            abs_error       = real_result - expected_result;
            if (abs_error < 0.0) abs_error = -abs_error;

            $display("[%0t ns] <--- Header: 0x%02h | Footer: 0x%02h", $time, rx_header, rx_footer);
            $display("[%0t ns] <--- Valor crudo (Hex): 0x%016h", $time, rx_payload_captured[63:0]);
            $display("             Parte Entera:       %0d", int_part);
            $display("             Parte Fraccionaria: %0d / 65536 (aprox. 0.%04d)", frac_part, (frac_part * 10000) / 65536);
            $display("             Resultado FPGA:     %0.4f", real_result);
            $display("             Resultado Teorico:  %0.4f", expected_result);
            $display("             Error Absoluto:     %0.6f", abs_error);

            if (rx_header == 8'hAA && rx_footer == 8'h55 && abs_error < 0.05)
                $display("             ===> RESULTADO: CORRECTO");
            else
                $display("             ===> RESULTADO: ERROR O FUERA DE TOLERANCIA");
        end
    endtask

    // 7. Secuencia de pruebas
    initial begin
        UART_RX             = 1'b1;
        RESET_N             = 1'b0;
        rx_payload_captured = 512'd0;
        rx_header           = 8'd0;
        rx_footer           = 8'd0;

        #100;
        RESET_N = 1'b1;
        #(BIT_PERIOD * 4); 
		
		// Monitor en tiempo real del estado de la FPGA
		always @(uut.state) begin
		    $display("[%0t ns] [DEBUG FPGA] Estado cambio a: %0d | rx_done=%b, rx_data=0x%02h, pn_done=%b", 
		             $time, uut.state, uut.rx_done, uut.rx_data, uut.pn_done);
		end

        // =========================================================
        // PRUEBA 1: n = 10 -> 10^(5/2) ˜ 316.2277
        // =========================================================
        fork
            send_uart_frame_32bit(32'd10);
            verify_result(32'd10);
        join

        #(BIT_PERIOD * 10);

        // =========================================================
        // PRUEBA 2: n = 50 -> 50^(5/2) ˜ 17677.6695
        // =========================================================
        fork
            send_uart_frame_32bit(32'd50);
            verify_result(32'd50);
        join

        #(BIT_PERIOD * 10);

        // =========================================================
        // PRUEBA 3: n = 85 -> 85^(5/2) ˜ 66610.6654
        // =========================================================
        fork
            send_uart_frame_32bit(32'd85);
            verify_result(32'd85);
        join

        #(BIT_PERIOD * 10);
        $display("\n--- Simulacion completada exitosamente ---");
        $stop;
    end

endmodule