% ============================================
% Script: bidireccional_matlab.m (n^(5/2))
% ============================================
clear
clc

PORT = "COM3";     % Cambiar por el puerto correspondiente a la FPGA / puente USB-UART
BAUD = 9600;

HEADER = uint8(hex2dec('AA'));
FOOTER = uint8(hex2dec('55'));
BYTES_PAYLOAD = 12;

s = serialport(PORT, BAUD, "Timeout", 2.0);
pause(2);
flush(s);

disp("Comunicacion bidireccional iniciada");
disp("La FPGA calculara n^(5/2) en punto fijo");

while true
    valor = input('Ingrese un numero entero (o numero negativo para salir): ');

    if valor < 0
        disp('Finalizando comunicacion.');
        break;
    end

    % 1. Empaquetar n en 4 bytes (little-endian) y armar la trama
    n_bytes = typecast(uint32(valor), 'uint8');
    trama_tx = [HEADER, n_bytes, FOOTER];

    flush(s);
    write(s, trama_tx, "uint8");
    disp(['Enviado: ', num2str(valor)]);

    pause(0.2);

    % 2. Verificar si llegaron bytes (se esperan al menos 14: 1 header + 12 payload + 1 footer)
    if s.NumBytesAvailable >= 14
        % Buscar sincronizacion con el delimitador 0xAA
        rx_header = read(s, 1, "uint8");
        while rx_header ~= HEADER && s.NumBytesAvailable > 0
            rx_header = read(s, 1, "uint8");
        end

        if rx_header == HEADER
            payload = read(s, BYTES_PAYLOAD, "uint8");
            rx_footer = read(s, 1, "uint8");

            if rx_footer == FOOTER
                % Reconstruir entero de 96 bits (LSB primero)
                pesos = 2 .^ (8 * (0:(BYTES_PAYLOAD - 1)));
                raw_int96 = sum(double(payload) .* pesos);

                % Convertir de formato Q80.16 (16 bits fraccionarios)
                resultado_fpga = raw_int96 / 65536.0;

                % Calculo teorico para comparacion
                teorico = (double(valor)^2) * sqrt(double(valor));

                disp(['Respuesta FPGA: ', num2str(resultado_fpga, '%.4f')]);
                disp(['Valor Teorico:  ', num2str(teorico, '%.4f')]);
            else
                disp('Error: Delimitador de fin 0x55 invalido.');
            end
        else
            disp('Error: No se encontro el delimitador 0xAA.');
        end
    else
        disp('No se recibio respuesta completa de la FPGA');
    end

    disp('--------------------------------------------');
end

clear s
