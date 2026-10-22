% =========================================================================
% Transmision y Recepcion UART: PC <-> FPGA DE10-Lite (Calculo n^(5/2))
% =========================================================================
clear; clc;

%% 1. Configuracion del puerto serie
puertoFPGA = "COM3";   % Cambia por tu puerto (ej. 'COM4' o '/dev/ttyUSB0')
baudRate   = 9600;

HEADER = uint8(hex2dec('AA'));
FOOTER = uint8(hex2dec('55'));
BYTES_PAYLOAD_RX = 12; % 96 bits en formato Q80.16 (12 bytes)

% Solicitar valor al usuario
prompt = 'Ingrese el valor entero de n (>= 0): ';
n_input = input(prompt);

if isempty(n_input) || n_input < 0
    error('Debe ingresar un numero entero positivo o cero.');
end

n = uint32(n_input);

%% 2. Apertura y preparacion del enlace UART
try
    s = serialport(puertoFPGA, baudRate, "Timeout", 2.0);
    flush(s);
    pause(0.2);
catch ME
    error("No se pudo abrir el puerto %s: %s", puertoFPGA, ME.message);
end

%% 3. Empaquetar y enviar el numero a la FPGA
% Descomponer n (32 bits) en 4 bytes (Little-Endian: LSB primero)
n_bytes = typecast(n, 'uint8');

% Estructura de trama de envio: [0xAA, b0, b1, b2, b3, 0x55]
tx_frame = [HEADER, n_bytes, FOOTER];

fprintf('\nEnviando n = %d a la FPGA...\n', n);
fprintf('Trama transmitida (Hex): %s\n', sprintf('%02X ', tx_frame));

flush(s);
write(s, tx_frame, "uint8");

%% 4. Recepcion de la respuesta de la FPGA
% Leer encabezado de inicio (0xAA)
rx_head = read(s, 1, "uint8");
if isempty(rx_head)
    clear s;
    error('Tiempo de espera agotado: La FPGA no respondio.');
elseif rx_head ~= HEADER
    clear s;
    error('Delimitador de inicio invalido recibido: 0x%02X (se esperaba 0xAA).', rx_head);
end

% Leer los 12 bytes del resultado de la operacion
rx_payload = read(s, BYTES_PAYLOAD_RX, "uint8");
if length(rx_payload) < BYTES_PAYLOAD_RX
    clear s;
    error('Trama de datos incompleta recibida de la FPGA.');
end

% Leer delimitador final (0x55)
rx_foot = read(s, 1, "uint8");
if isempty(rx_foot) || rx_foot ~= FOOTER
    clear s;
    error('Delimitador de fin invalido recibido: 0x%02X (se esperaba 0x55).', rx_foot);
end

%% 5. Reconstruccion y conversion de punto fijo Q80.16
% Pesos exponenciales para recomponer el entero de 96 bits (LSB primero)
pesos = 2 .^ (8 * (0:(BYTES_PAYLOAD_RX - 1)));
raw_int96 = sum(double(rx_payload) .* pesos);

% Conversion dividiendo entre 2^16 (16 bits fraccionarios)
resultado_fpga = raw_int96 / 65536.0;

% Calculo teorico analitico para comparacion
resultado_teorico = (double(n)^2) * sqrt(double(n));
error_abs = abs(resultado_fpga - resultado_teorico);

%% 6. Despliegue de resultados
fprintf('\n================== RESULTADOS ==================\n');
fprintf('Payload recibido (Hex): %s\n', sprintf('%02X ', rx_payload));
fprintf('Resultado FPGA (Real):  %.6f\n', resultado_fpga);
fprintf('Valor Teorico n^(5/2):  %.6f\n', resultado_teorico);
fprintf('Error Absoluto:         %.6e\n', error_abs);
fprintf('================================================\n');

% Cerrar conexion de manera segura
clear s;
