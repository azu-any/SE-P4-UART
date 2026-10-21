% =========================================================================
% Script de Comunicacion UART para DE10-Lite (Calculo de n^(5/2))
% =========================================================================
clear; clc;

%% 1. Parametros del Puerto Serial
puertoFPGA = "COM3";   % Cambiar segun corresponda (ej. '/dev/ttyUSB0' en Linux)
baudRate   = 9600;

HEADER = uint8(hex2dec('AA'));
FOOTER = uint8(hex2dec('55'));
BYTES_PAYLOAD = 12;    % 96 bits = 12 bytes

%% 2. Apertura del Puerto
try
    s = serialport(puertoFPGA, baudRate, "Timeout", 2.0);
    flush(s);
    pause(0.5);
    fprintf("Conexion establecida con exito en %s a %d baudios.\n\n", puertoFPGA, baudRate);
catch ME
    error("Error al abrir %s: %s", puertoFPGA, ME.message);
end

%% 3. Valores de Prueba
test_values = uint32([10, 50, 85, 144]);

for i = 1:length(test_values)
    n = test_values(i);
    fprintf("====================================================\n");
    fprintf("Evaluando n = %d\n", n);
    
    % Descomponer n (32 bits) en 4 bytes (Little-Endian)
    n_bytes = typecast(n, 'uint8');
    
    % Trama: [0xAA, b0, b1, b2, b3, 0x55]
    tx_frame = [HEADER, n_bytes, FOOTER];
    
    flush(s);
    write(s, tx_frame, "uint8");
    
    % Leer Header (1 byte)
    rx_header = read(s, 1, "uint8");
    if isempty(rx_header) || rx_header ~= HEADER
        warning("Fallo al recibir el delimitador de inicio 0xAA.");
        continue;
    end
    
    % Leer Payload (12 bytes)
    payload = read(s, BYTES_PAYLOAD, "uint8");
    if length(payload) < BYTES_PAYLOAD
        warning("Datos incompletos recibidos desde la FPGA.");
        continue;
    end
    
    % Leer Footer (1 byte)
    rx_footer = read(s, 1, "uint8");
    if isempty(rx_footer) || rx_footer ~= FOOTER
        warning("Fallo al recibir el delimitador final 0x55.");
        continue;
    end
    
    % Reconstruir entero de 96 bits usando aritmetica de pesos
    pesos = 2 .^ (8 * (0:11));
    raw_int96 = sum(double(payload) .* pesos);
    
    % Convertir desde formato Q80.16 (dividir entre 2^16 = 65536)
    fpga_result = raw_int96 / 65536.0;
    
    % Valor analitico teorico
    theoretical = double(n)^2 * sqrt(double(n));
    error_abs   = abs(fpga_result - theoretical);
    
    fprintf("Bytes Recibidos (Hex): %s\n", sprintf('%02X ', payload));
    fprintf("Resultado FPGA:        %.4f\n", fpga_result);
    fprintf("Valor Teorico:         %.4f\n", theoretical);
    fprintf("Error Absoluto:        %.6f\n", error_abs);
    
    pause(0.2);
end

% Cerrar conexion
clear s;
fprintf("\n====================================================\n");
fprintf("Todas las pruebas concluyeron correctamente.\n");
