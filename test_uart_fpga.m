% =========================================================================
% Script: test_uart_fpga.m
% Descripcion: Envia un entero n de 32 bits desde MATLAB -> ESP32 -> FPGA,
%              recibe el resultado de 96 bits en formato Q80.16 (n^(5/2))
%              y lo compara contra el valor teorico.
% =========================================================================
clear;
clc;

% 1. Configuracion del puerto serial
PORT = "COM3";       % Cambiar por el puerto correspondiente
BAUD = 9600;          % Debe coincidir con ESP32 y FPGA

HEADER = uint8(hex2dec('AA'));
FOOTER = uint8(hex2dec('55'));
BYTES_PAYLOAD = 12;   % Resultado de 96 bits Q80.16

% 2. Abrir conexion serial
disp("Abriendo puerto serial y esperando inicio del ESP32...");
s = serialport(PORT, BAUD, "Timeout", 2.0);
pause(2.5);
flush(s);

disp("==========================================================");
disp(" Comunicacion MATLAB <-> ESP32 <-> FPGA");
disp(" Calculo seleccionable: n^(p/2)");
disp("==========================================================");

while true
    % 3. Solicitar n
    valor = input('\nIngrese un entero n >= 0 (negativo para salir): ');

    if isempty(valor)
        continue;
    end

    if ~isnumeric(valor) || ~isscalar(valor) || ~isfinite(valor)
        disp('Error: introduzca un valor numerico finito.');
        continue;
    end

    if valor < 0
        disp('Finalizando comunicacion y cerrando puerto...');
        break;
    end

    if valor ~= fix(valor) || valor > double(intmax('uint32'))
        disp('Error: n debe ser entero entre 0 y 2^32-1.');
        continue;
    end

    % 4. Solicitar el numerador p. El denominador permanece fijo en 2.
    while true
        p_num = input('Ingrese p (ej. 5 para 5/2, 7 para 7/2): ');

        if ~isempty(p_num) && isnumeric(p_num) && isscalar(p_num) && ...
                isfinite(p_num) && p_num == fix(p_num) && ...
                p_num >= 1 && p_num <= 255
            break;
        end

        disp('Error: p debe ser un entero entre 1 y 255.');
    end

    n_uint32 = uint32(valor);
    p_byte = uint8(p_num);
    n_double = double(n_uint32);

    % 5. Comprobar que el resultado teorico cabe en Q80.16.
    % Q80.16 tiene 80 bits para la parte entera y 16 para la fraccion.
    teorico = n_double ^ (double(p_byte) / 2.0);
    if ~isfinite(teorico) || teorico >= 2^80
        fprintf(['Error: n^(%u/2) no cabe en la parte entera del formato ' ...
                 'Q80.16. Pruebe con un n o p menor.\n'], p_byte);
        continue;
    end

    % 6. Empaquetar n en Little-Endian y crear la trama de 7 bytes.
    n_bytes = typecast(n_uint32, 'uint8');
    trama_tx = [HEADER, n_bytes, p_byte, FOOTER];

    flush(s);
    write(s, trama_tx, "uint8");

    fprintf('Enviado n = %u | Exponente = %u/2\n', n_uint32, p_byte);
    fprintf('Trama TX (7 bytes): %s\n', sprintf('0x%02X ', trama_tx));

    % 7. Buscar la cabecera de respuesta 0xAA.
    header_found = false;
    tiempo_limite = 1.5;
    t_inicio = tic;

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

    % 8. Leer los 12 bytes de resultado y el byte final.
    if header_found
        bytes_restantes = BYTES_PAYLOAD + 1;

        while s.NumBytesAvailable < bytes_restantes && toc(t_inicio) < tiempo_limite
            pause(0.002);
        end

        if s.NumBytesAvailable >= bytes_restantes
            payload = read(s, BYTES_PAYLOAD, "uint8");
            rx_footer = read(s, 1, "uint8");
            trama_rx = [HEADER, payload, rx_footer];

            fprintf('Trama RX (14 bytes): %s\n', sprintf('0x%02X ', trama_rx));

            if rx_footer == FOOTER
                % Reconstruir el valor raw de 96 bits desde bytes Little-Endian.
                % Se usa double para mostrarlo; para valores grandes, double
                % no conserva todos los bits de un entero de 96 bits.
                pesos = 2 .^ (8 * (0:(BYTES_PAYLOAD - 1)));
                raw_int96 = sum(double(payload) .* pesos);

                frac_raw = double(payload(1)) + double(payload(2)) * 256.0;
                int_part = sum(double(payload(3:12)) .* (2 .^ (8 * (0:9))));
                resultado_fpga = raw_int96 / 65536.0;
                error_abs = abs(resultado_fpga - teorico);

                disp('----------------------------------------------------------');
                fprintf('Numero n:                  %u\n', n_uint32);
                fprintf('Exponente:                 %u/2\n', p_byte);
                fprintf('Parte Entera (FPGA):       %.0f\n', int_part);
                fprintf('Parte Fraccionaria (FPGA): %.0f / 65536 (%.6f)\n', ...
                        frac_raw, frac_raw / 65536.0);
                fprintf('Resultado FPGA (Q80.16):   %.6f\n', resultado_fpga);
                fprintf('Valor Teorico (MATLAB):    %.6f\n', teorico);
                fprintf('Error Absoluto:            %.6e\n', error_abs);
                disp('----------------------------------------------------------');
            else
                fprintf(['Error: delimitador final invalido: 0x%02X; ' ...
                         'se esperaba 0x55.\n'], rx_footer);
            end
        else
            fprintf(['Error: se recibio 0xAA pero no llegaron todos los bytes. ' ...
                     'Disponibles: %d.\n'], s.NumBytesAvailable);
        end
    else
        if s.NumBytesAvailable > 0
            basura = read(s, s.NumBytesAvailable, "uint8");
            fprintf('No se encontro 0xAA. Bytes: %s\n', sprintf('0x%02X ', basura));
        else
            disp('Error: timeout; no se recibio respuesta de la FPGA.');
        end
    end
end

% 9. Cerrar y liberar el puerto serial.
clear s;
disp('Puerto serial liberado correctamente.');
