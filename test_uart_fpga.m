% =========================================================================
% Script: test_uart_fpga.m
% Descripcion: Envia un entero n de 32 bits desde MATLAB -> ESP32 -> FPGA,
%              recibe el resultado de 96 bits en formato Q80.16 (n^(5/2))
%              y lo compara contra el valor teorico.
% =========================================================================
clear
clc

% 1. Configuracion del puerto serial
PORT = "COM8";     
BAUD = 9600;       % Debe coincidir exactamente con Serial.begin(9600) del ESP32

HEADER        = uint8(hex2dec('AA'));
FOOTER        = uint8(hex2dec('55'));
BYTES_PAYLOAD = 12; % 96 bits en formato Q80.16 = 12 bytes

% 2. Abrir conexion serial y esperar el reinicio automatico del ESP32
disp("Abriendo puerto serial y esperando inicio del ESP32...");
s = serialport(PORT, BAUD, "Timeout", 2.0);
pause(2.5); % El ESP32 tarda ~2s en arrancar al abrir el puerto serial
flush(s);   % Limpiar mensajes del bootloader del ESP32

disp("==========================================================");
disp(" Comunicacion bidireccional iniciada (MATLAB <-> ESP32 <-> FPGA)");
disp(" Calculando f(n) = n^(p/2) en formato punto fijo Q80.16");
disp("==========================================================");

while true
    valor = input('\nIngrese la base entera n >= 0 (o negativo para salir): ');

    if isempty(valor)
        continue;
    end

    if valor < 0
        disp('Finalizando comunicacion y cerrando puerto...');
        break;
    end

    exponente = input("Ingrese el exponente entero p >= 0 (para calcular n^(p/2)): ");

    if exponente < 0 || isempty(exponente)
        disp('Error leyendo exponente intente de nuevo.');
        continue;
    end

    % 3. Empaquetar n en 4 bytes (Little-Endian) y armar la trama de 6 bytes
    n_uint32 = uint32(floor(valor));
    p_uint8 = uint8(floor(exponente));
    n_bytes  = typecast(n_uint32, 'uint8'); % [LSB, b1, b2, MSB]
    trama_tx = [HEADER, n_bytes, p_uint8, FOOTER];

    % Limpiar buffer de entrada antes de enviar nueva solicitud
    flush(s);


    % 4. Transmitir trama de 6 bytes hacia el ESP32 -> FPGA
    write(s, trama_tx, "uint8");
    fprintf('Enviado n = %u | Trama TX (6 bytes): %s\n', ...
            n_uint32, sprintf('0x%02X ', trama_tx));

    % 5. Paso 1 de Recepcion: Buscar el delimitador de inicio 0xAA
    header_found  = false;
    tiempo_limite = 1.5; % 1.5 segundos de timeout maximo
    t_inicio      = tic;

    while toc(t_inicio) < tiempo_limite
        if s.NumBytesAvailable > 0
            rx_byte = read(s, 1, "uint8");
            if rx_byte == HEADER
                header_found = true;
                break;
            else
                fprintf('  [Aviso] Byte descartado antes de cabecera: 0x%02X\n', rx_byte);
            end
        else
            pause(0.002);
        end
    end

    % 6. Paso 2 de Recepcion: Leer los 13 bytes restantes (12 payload + 1 footer)
    if header_found
        % Esperar a que lleguen los 13 bytes completos despues de 0xAA
        while s.NumBytesAvailable < (BYTES_PAYLOAD + 1) && toc(t_inicio) < tiempo_limite
            pause(0.002);
        end

        if s.NumBytesAvailable >= (BYTES_PAYLOAD + 1)
            payload   = read(s, BYTES_PAYLOAD, "uint8");
            rx_footer = read(s, 1, "uint8");

            trama_rx = [HEADER, payload, rx_footer];
            fprintf('Recibido      | Trama RX (14 bytes): %s\n', ...
                    sprintf('0x%02X ', trama_rx));

            if rx_footer == FOOTER
                % 7. Reconstruir el numero de 96 bits en formato Q80.16 (Little-Endian)
                % Bytes 1..2  (indices 1..2 en MATLAB) = Parte fraccionaria (16 bits)
                % Bytes 3..12 (indices 3..12 en MATLAB) = Parte entera (80 bits)
                pesos     = 2 .^ (8 * (0:(BYTES_PAYLOAD - 1)));
                raw_int96 = sum(double(payload) .* pesos);

                % Separar parte fraccionaria y entera para inspeccion
                frac_raw = double(payload(1)) + double(payload(2)) * 256.0;
                int_part = sum(double(payload(3:12)) .* (2 .^ (8 * (0:9))));

                % Convertir de Q80.16 a punto flotante dividiendo entre 2^16 (65536)
                resultado_fpga = raw_int96 / 65536.0;

                % Calculo teorico en doble precision
                n_double = double(n_uint32);
                p_double = double(p_uint8);
                teorico  = n_double ^ (p_double / 2.0);
                error_abs = abs(resultado_fpga - teorico);

                disp('----------------------------------------------------------');
                fprintf('  Parte Entera (FPGA):       %.0f\n', int_part);
                fprintf('  Parte Fraccionaria (FPGA): %.0f / 65536 (%.4f)\n', ...
                        frac_raw, frac_raw / 65536.0);
                fprintf('  Resultado FPGA (Q80.16):   %.4f\n', resultado_fpga);
                fprintf('  Valor Teorico (MATLAB):    %.4f\n', teorico);
                fprintf('  Error Absoluto:            %.6e\n', error_abs);
                disp('----------------------------------------------------------');
            else
                fprintf('Error: Delimitador de fin invalido (se recibio 0x%02X, se esperaba 0x55).\n', rx_footer);
            end
        else
            fprintf('Error: Se recibio 0xAA pero solo llegaron %d de los 13 bytes restantes.\n', s.NumBytesAvailable);
        end
    else
        if s.NumBytesAvailable > 0
            basura = read(s, s.NumBytesAvailable, "uint8");
            fprintf('Error: No se encontro 0xAA. Bytes en buffer: %s\n', sprintf('0x%02X ', basura));
        else
            disp('Error: Timeout agotado (0 bytes recibidos desde el ESP32/FPGA).');
        end
    end
end

% Cerrar y liberar el puerto serial
clear s
disp('Puerto serial liberado correctamente.');
