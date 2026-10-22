% ============================================
% Script: bidireccional_matlab_n52.m
% ============================================
clear
clc

PORT = "COM3";     % Puerto serial
BAUD = 9600;

HEADER = uint8(hex2dec('AA'));
FOOTER = uint8(hex2dec('55'));

s = serialport(PORT, BAUD);
pause(2);
flush(s);

disp("Comunicacion bidireccional iniciada");
disp("Calculando n^(5/2) en la FPGA DE10-Lite");

while true
    valor = input('\nIngrese un numero entero (n >= 0): ');

    if valor < 0
        disp('Saliendo...');
        break;
    end

    % 1. Empaquetar n en 4 bytes (little-endian) y armar la trama
    n_bytes = typecast(uint32(valor), 'uint8');
    trama_tx = [HEADER, n_bytes, FOOTER];

    flush(s);
    write(s, trama_tx, "uint8");
    disp(['Enviado n = ', num2str(valor), ' | Trama: ', sprintf('0x%02X ', trama_tx)]);

    % 2. Pausa para dar tiempo a la transmision serial (14 bytes @ 9600 baud = ~15 ms)
    pause(0.2);

    % 3. Inspeccionar qué llegó al bufer
    bytes_disponibles = s.NumBytesAvailable;
    fprintf('Bytes recibidos en el puerto: %d\n', bytes_disponibles);

    if bytes_disponibles > 0
        % Leer todos los bytes que devolvio la FPGA
        datos_rx = read(s, bytes_disponibles, "uint8");
        fprintf('Datos crudos (Hex): %s\n', sprintf('0x%02X ', datos_rx));
        fprintf('Datos crudos (Dec): %s\n', sprintf('%d ', datos_rx));

        % Diagnóstico rápido:
        if datos_rx(1) == uint8(hex2dec('AB'))
            disp('--> AVISO: La FPGA respondio 0xAB (171). Sigue cargado el diseno de prueba (+1).');
            disp('--> Debes programar el archivo .sof de "top_uart_de10lite" en Quartus Programmer.');
        elseif datos_rx(1) == HEADER && bytes_disponibles >= 14
            % Trama completa de n^(5/2)
            payload = datos_rx(2:13);
            delimitador_fin = datos_rx(14);

            if delimitador_fin == FOOTER
                % Reconstruir entero de 96 bits (LSB primero)
                pesos = 2 .^ (8 * (0:11));
                raw_int96 = sum(double(payload) .* pesos);

                % Convertir de formato Q80.16 (dividir entre 2^16 = 65536)
                fpga_res = raw_int96 / 65536.0;
                teorico  = (double(valor)^2) * sqrt(double(valor));

                disp('--------------------------------------------');
                disp(['Respuesta FPGA: ', num2str(fpga_res, '%.4f')]);
                disp(['Valor Teorico:  ', num2str(teorico, '%.4f')]);
                disp(['Error Absoluto: ', num2str(abs(fpga_res - teorico), '%.6e')]);
                disp('--------------------------------------------');
            else
                disp('Error: Delimitador final 0x55 incorrecto.');
            end
        else
            disp('Trama incompleta o formato desconocido.');
        end
    else
        disp('No se recibio respuesta de la FPGA');
    end
end

clear s
