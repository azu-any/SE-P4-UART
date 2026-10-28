# Práctica 4: Coprocesador Matemático FPGA $n^{p/2}$ con Interfaz UART y Pasarela ESP32

**Institución:** Universidad de las Américas Puebla (UDLAP)  
**Curso:** Sistemas Embebidos (9º Semestre) — Parcial 3  
**Plataforma Hardware:** Tarjeta FPGA Terasic DE10-Lite (Intel MAX 10 `10M50DAF484C7G`)  
**Microcontrolador Intermedio:** Espressif ESP32 (Puente transparente USB-UART con LED de diagnóstico)  
**Consola de Usuario:** MATLAB R2020a+ (`test_uart_fpga.m`)  
**Frecuencia de Reloj FPGA:** 50 MHz (`CLOCK_50`)  
**Protocolo Serial:** UART a 9600 Baudios (8N1: 8 bits de datos, sin paridad, 1 bit de parada)

---

## 1. Descripción General del Proyecto

Este proyecto implementa un acelerador de hardware (coprocesador aritmético parametrizado) en Verilog HDL sobre una FPGA Intel MAX 10. El sistema calcula de forma digital la función matemática generalizada de potencias fraccionarias:

$$f(n, p) = n^{p/2} = n^{\lfloor p/2 \rfloor} \cdot \left(\sqrt{n}\right)^{p \pmod 2}$$

donde:
- $n$ es la **base entera sin signo** de 32 bits ($n \in [0, \, 2^{32}-1]$).
- $p$ es el **exponente entero sin signo** de 8 bits ($p \in [0, \, 255]$).
- El resultado se entrega en un bus de **96 bits** en formato de punto fijo **Q80.16** (80 bits para la parte entera y 16 bits para la parte fraccionaria).

### Arquitectura de Comunicación en Tres Niveles
El sistema opera a través de un esquema cliente-pasarela-coprocesador:

```mermaid
flowchart TD
    subgraph PC["Estación de Trabajo / Computadora (PC)"]
        MATLAB["MATLAB (test_uart_fpga.m)\nConsola Interactiva de Usuario\nIngreso de n (base 32b) y p (exp 8b)"]
    end

    subgraph BRIDGE["Pasarela de Comunicación (ESP32)"]
        ESP32["ESP32 (arduino.cpp)\nBridge Transparente USB <-> UART (9600 Baud)\nLED Diagnóstico de Actividad (GPIO 2)"]
    end

    subgraph FPGA_DE10["Tarjeta FPGA Terasic DE10-Lite (MAX 10 - 10M50DAF484C7G)"]
        subgraph TOP["top_uart_de10lite"]
            FSM_TOP["FSM Principal del Sistema\nRecepción, Secuenciación y Empaquetado"]
            
            subgraph UART_CORE["Subsistema UART (9600 Baud, 8N1)"]
                RX["uart_rx.v\nReceptor Serial Asíncrono"]
                TX["uart_tx.v\nTransmisor Serial Asíncrono"]
            end
            
            subgraph COPROCESSOR["Coprocesador Matemático Parametrizado n^(p/2)"]
                PN["process_number.v\nFSM de Potencia Entera & Combinación Final\nCalcula n^(floor(p/2)) y combina con raíz"]
                SQRT["square_root.v\nFSM Raíz Cuadrada Dígito a Dígito (Q16.16)\n32 Iteraciones en Paralelo con la Potencia"]
            end
        end
    end

    MATLAB <-->|"USB Serial (COM8, 9600 Baud)\nTrama TX: 7 Bytes | Trama RX: 14 Bytes"| ESP32
    ESP32 <-->|"Hardware UART (RXD2=16, TXD2=17, 9600 Baud)\nTrama TX: 7 Bytes | Trama RX: 14 Bytes"| TOP
    
    RX -->|"rx_data[7:0], rx_done"| FSM_TOP
    FSM_TOP -->|"start, data_in[31:0], p_in[7:0]"| PN
    PN -->|"start, data_in[31:0]"| SQRT
    SQRT -->|"sqrt_out[31:0] (Q16.16), done"| PN
    PN -->|"data_out[95:0] (Q80.16), done"| FSM_TOP
    FSM_TOP -->|"tx_data[7:0], tx_start"| TX
    TX -->|"busy"| FSM_TOP
```

---

## 2. Especificación Matemática y Formato Numérico

### 2.1 Descomposición del Cálculo de $n^{p/2}$
Para optimizar el uso de recursos y latencia en la FPGA, el exponente $p$ de 8 bits se divide en dos componentes mediante aritmética de bits:
1. **Parte entera del exponente:** $\lfloor p / 2 \rfloor = p \gg 1$ (bits `p_in[7:1]`). Determina cuántas veces consecutivas se multiplica la base $n$ por el acumulador entero `pow_acc` (80 bits).
2. **Componente fraccionario (raíz cuadrada):** Determinado por el bit menos significativo $p \pmod 2 = p[0]$ (`p_is_odd`):
   - **Si $p$ es par (`p_is_odd == 0`):** El resultado es un entero puro $n^{p/2}$. No requiere multiplicación con la raíz; el acumulador entero `pow_acc` se alinea a la izquierda insertando 16 ceros en los bits fraccionarios:
     $$\text{data\_out} = \{\text{pow\_acc}[79:0], \, 16'd0\}$$
   - **Si $p$ es impar (`p_is_odd == 1`):** El resultado requiere el factor fraccionario $\sqrt{n}$. Se realiza la multiplicación de 80 bits enteros por 32 bits en formato Q16.16:
     $$\text{data\_out} = \text{pow\_acc}[79:0] \times \text{sqrt\_res}[31:0]$$
3. **Ejecución Concurrente:** El módulo [`process_number`](file:///Users/Azu/Desktop/School/UDLAP/9o/Sistemas%20Embebidos/Parcial3/Practica4/process_number.v) dispara el cálculo de raíz cuadrada en el módulo [`square_root`](file:///Users/Azu/Desktop/School/UDLAP/9o/Sistemas%20Embebidos/Parcial3/Practica4/square_root.v) en paralelo con las multiplicaciones de potencia entera, minimizando la latencia total.
4. **Manejo de Casos Límite:**
   - Si $n = 0$ y $p > 0$, el módulo entrega de inmediato `data_out = 96'd0`.
   - Si $p = 0$, $n^0 = 1$ en formato Q80.16 (`0x00000000000000010000`).
   - Para el caso clásico de la práctica ($p = 5$), el sistema evalúa $n^{5/2} = n^2 \cdot \sqrt{n}$.

### 2.2 Representación en Punto Fijo (Q80.16)
El resultado final consta de 96 bits organizados de la siguiente forma:

| Rango de Bits | Ancho | Tipo | Descripción |
|---|---|---|---|
| `[95:16]` | 80 bits | Entero sin signo | Representa la parte entera de $n^{p/2}$ |
| `[15:0]` | 16 bits | Fraccionario | Representa la parte decimal $\frac{\text{valor}}{2^{16}} = \frac{\text{valor}}{65536}$ (resolución $\approx 1.5258 \times 10^{-5}$) |

Para obtener el valor decimal en software:
$$\text{Valor Decimal} = \frac{\text{data\_out}[95:0]}{65536.0}$$

---

## 3. Protocolo de Comunicación UART

Tanto el enlace USB de la PC con el ESP32 como el canal físico entre el ESP32 y la FPGA operan a **9600 baudios**, formato **8N1** (8 bits de datos, sin paridad, 1 bit de parada).

### 3.1 Trama de Solicitud (MATLAB $\to$ ESP32 $\to$ FPGA) — 7 Bytes
El Host transmite un paquete de 7 bytes delimitado por cabecera y pie con el operando de 32 bits en formato Little-Endian y el exponente de 8 bits:

```
+----------+----------+----------+----------+----------+----------+----------+
| Byte 0   | Byte 1   | Byte 2   | Byte 3   | Byte 4   | Byte 5   | Byte 6   |
+----------+----------+----------+----------+----------+----------+----------+
| 0xAA     | n[7:0]   | n[15:8]  | n[23:16] | n[31:24] | p[7:0]   | 0x55     |
| (Header) | Base LSB | Base     | Base     | Base MSB | Exponente| (Footer) |
+----------+----------+----------+----------+----------+----------+----------+
```

### 3.2 Trama de Respuesta (FPGA $\to$ ESP32 $\to$ MATLAB) — 14 Bytes
La FPGA responde con una trama de 14 bytes conteniendo el resultado de 96 bits en formato Little-Endian:

```
+----------+-------------------------------------------------------------+----------+
| Byte 0   | Bytes 1 .. 12 (96 bits en Q80.16, Little-Endian)           | Byte 13  |
+----------+-------------------------------------------------------------+----------+
| 0xAA     | Byte 1: pn_result[7:0]   (Fracción LSB)                     | 0x55     |
| (Header) | Byte 2: pn_result[15:8]  (Fracción MSB)                     | (Footer) |
|          | Byte 3 .. 12: pn_result[23:16] a [95:88] (Parte Entera)    |          |
+----------+-------------------------------------------------------------+----------+
```

---

## 4. Máquinas de Estados Finitas (FSM)

Todos los módulos secuenciales clave del hardware implementan Máquinas de Estados Finitas (FSM). A continuación se documenta en detalle el funcionamiento, transiciones y diagrama de cada una.

---

### 4.1 FSM del Módulo Superior (`top_uart_de10lite.v`)

El módulo [`top_uart_de10lite`](file:///Users/Azu/Desktop/School/UDLAP/9o/Sistemas%20Embebidos/Parcial3/Practica4/top_uart_de10lite.v) actúa como el secuenciador maestro del sistema. Implementa una FSM de **8 estados** (registro `state` de 3 bits) encargada de validar la trama entrante, disparar el coprocesador y serializar los 12 bytes del resultado hacia el transmisor UART.

#### Diagrama de Estados:

```mermaid
stateDiagram-v2
    [*] --> S_IDLE
    
    S_IDLE --> S_RECEIVE: rx_done && rx_data == 0xAA\n[complete_data <= 0, index <= 0]
    S_IDLE --> S_IDLE: rx_done && rx_data != 0xAA
    
    S_RECEIVE --> S_RECEIVE: rx_done && index < 32\n[complete_data[index +: 8] <= rx_data, index <= index + 8]
    S_RECEIVE --> S_WAIT_MATH: rx_done && index >= 32 && rx_data == 0x55\n[pn_start <= 1, index <= 0]
    S_RECEIVE --> S_IDLE: rx_done && index >= 32 && rx_data != 0x55\n(Trama inválida: abortar)
    
    S_WAIT_MATH --> S_SEND_HEADER: pn_done == 1\n[index <= 0]
    S_WAIT_MATH --> S_WAIT_MATH: pn_done == 0
    
    S_SEND_HEADER --> S_PROCESS: !tx_busy\n[tx_data <= 0xAA, tx_start <= 1]
    
    S_PROCESS --> S_WAIT_TX: !tx_busy && !tx_start\n[tx_data <= pn_result[index +: 8], tx_start <= 1]
    
    S_WAIT_TX --> S_PROCESS: !tx_busy && !tx_start && index < 88\n[index <= index + 8]
    S_WAIT_TX --> S_SEND_FOOTER: !tx_busy && !tx_start && index >= 88
    
    S_SEND_FOOTER --> S_WAIT_FOOTER: !tx_busy && !tx_start\n[tx_data <= 0x55, tx_start <= 1]
    
    S_WAIT_FOOTER --> S_IDLE: !tx_busy && !tx_start\n(Trama transmitida con éxito)
```

#### Descripción de Estados:

| Estado | Código | Función / Comportamiento |
|---|---|---|
| `S_IDLE` | `3'b000` | Modo de espera. Al recibir el byte delimitador `0xAA` vía `rx_done`, reinicia el índice y el búfer de entrada, transicionando a `S_RECEIVE`. |
| `S_RECEIVE` | `3'b001` | Acumula bytes de datos de forma indexada (`index +: 8`). Al capturar los datos requeridos, valida el byte de cierre `0x55`. Si es correcto, emite el pulso `pn_start <= 1` y avanza a `S_WAIT_MATH`; en caso contrario, descarta la trama regresando a `S_IDLE`. |
| `S_WAIT_MATH` | `3'b010` | Espera el pulso de finalización `pn_done` del coprocesador `process_number`. Al detectarlo, reinicia `index <= 0` y avanza a `S_SEND_HEADER`. |
| `S_SEND_HEADER` | `3'b011` | Cuando el transmisor UART está disponible (`!tx_busy`), carga `0xAA` en `tx_data`, pulsa `tx_start <= 1` y avanza a `S_PROCESS`. |
| `S_PROCESS` | `3'b100` | Carga el byte correspondiente `pn_result[index +: 8]` en `tx_data`, pulsa `tx_start <= 1` y pasa a `S_WAIT_TX`. |
| `S_WAIT_TX` | `3'b101` | Aguarda que el transmisor finalice el byte actual. Si `index < 88`, suma 8 al índice y regresa a `S_PROCESS` para el siguiente byte; si los 12 bytes ya fueron cargados (`index == 88`), avanza a `S_SEND_FOOTER`. |
| `S_SEND_FOOTER` | `3'b110` | Carga `0x55` en `tx_data`, pulsa `tx_start <= 1` y avanza a `S_WAIT_FOOTER`. |
| `S_WAIT_FOOTER` | `3'b111` | Aguarda a que el bit de parada del byte de cierre termine de transmitirse y retorna al estado de reposo `S_IDLE`. |

---

### 4.2 FSM del Coprocesador Parametrizado (`process_number.v`)

El módulo [`process_number`](file:///Users/Azu/Desktop/School/UDLAP/9o/Sistemas%20Embebidos/Parcial3/Practica4/process_number.v) calcula $n^{p/2}$ combinando acumulación de potencias enteras y la raíz cuadrada concurrente. Implementa una FSM de **4 estados** (registro `state` de 2 bits).

#### Diagrama de Estados:

```mermaid
stateDiagram-v2
    [*] --> S_IDLE
    
    S_IDLE --> S_CALC: start == 1\n[pow_acc <= 1, exp_cnt <= p_in[7:1], p_is_odd <= p_in[0], sqrt_start <= 1, sqrt_ready <= 0]
    S_IDLE --> S_IDLE: start == 0
    
    S_CALC --> S_CALC: exp_cnt > 0\n[pow_acc <= pow_acc * data_in, exp_cnt <= exp_cnt - 1]
    S_CALC --> S_MULT: exp_cnt == 0 && (sqrt_ready || sqrt_done)
    
    S_MULT --> S_DONE: data_in == 0 && p_in != 0\n[data_out <= 0]
    S_MULT --> S_DONE: p_is_odd == 1\n[data_out <= pow_acc * sqrt_res (Q80.16)]
    S_MULT --> S_DONE: p_is_odd == 0\n[data_out <= {pow_acc, 16'd0} (Q80.16)]
    
    S_DONE --> S_IDLE: done <= 1
```

#### Descripción de Estados:

| Estado | Código | Función / Comportamiento |
|---|---|---|
| `S_IDLE` | `2'b00` | Reposo. Al recibir `start`: inicializa `pow_acc <= 80'd1`, extrae el conteo de multiplicaciones enteras `exp_cnt <= p_in[7:1]`, almacena la bandera de paridad `p_is_odd <= p_in[0]`, dispara la raíz cuadrada concurrente `sqrt_start <= 1` y pasa a `S_CALC`. |
| `S_CALC` | `2'b01` | **Cálculo concurrente:** Si `exp_cnt > 0`, multiplica en cada ciclo `pow_acc <= pow_acc * data_in` y decrementa `exp_cnt`. Simultáneamente, monitorea la señal `sqrt_done` del módulo de raíz cuadrada registrándola en `sqrt_ready`. Cuando **ambos** procesos terminan (`exp_cnt == 0` y `sqrt_ready \|\| sqrt_done`), transiciona a `S_MULT`. |
| `S_MULT` | `2'b10` | **Resolución del formato final:** Si $n = 0$ ($p > 0$), asigna `data_out <= 0`. Si $p$ es impar, multiplica el acumulador entero de 80 bits por el resultado de raíz Q16.16 de 32 bits (`pow_acc * sqrt_res`). Si $p$ es par, alinea el entero a Q80.16 desplazándolo 16 bits (`{pow_acc, 16'd0}`). Avanza a `S_DONE`. |
| `S_DONE` | `2'b11` | Genera un pulso activo en alto de un ciclo de reloj en `done <= 1` y retorna a `S_IDLE`. |

---

### 4.3 FSM del Módulo de Raíz Cuadrada (`square_root.v`)

El módulo [`square_root`](file:///Users/Azu/Desktop/School/UDLAP/9o/Sistemas%20Embebidos/Parcial3/Practica4/square_root.v) implementa el algoritmo digital de extracción de raíz cuadrada no restaurativa por sustracciones y desplazamientos bit a bit. Utiliza una FSM de **3 estados** (registro `state` de 2 bits) que ejecuta 32 iteraciones exactas.

#### Diagrama de Estados:

```mermaid
stateDiagram-v2
    [*] --> S_IDLE
    
    S_IDLE --> S_CALC: start == 1\n[rem_data <= {data_in, 32'd0}, r <= 0, q <= 0, iter <= 31]
    S_IDLE --> S_IDLE: start == 0
    
    S_CALC --> S_CALC: iter > 0\n[r_next >= test_val ? (r <= r_next - test_val, q <= {q[30:0], 1'b1}) : (r <= r_next, q <= {q[30:0], 1'b0})]\n[rem_data <= {rem_data[61:0], 2'b00}, iter <= iter - 1]
    
    S_CALC --> S_DONE: iter == 0\n(32 iteraciones completas)
    
    S_DONE --> S_IDLE: sqrt_out <= q, done <= 1
```

#### Fundamento del Algoritmo:
1. **Extensión Fraccionaria:** Se concatenan 32 ceros a la derecha del número de entrada (`rem_data <= {data_in, 32'd0}`), lo que asegura una resolución de 16 bits enteros y 16 bits fraccionarios en el resultado.
2. **Prueba por Pares de Bits:** En cada iteración:
   - `r_next = {r[31:0], rem_data[63:62]}`
   - `test_val = {q, 2'b01}`
   - Si `r_next >= test_val`, el nuevo bit de raíz es `1` y `r <= r_next - test_val`.
   - Si `r_next < test_val`, el nuevo bit de raíz es `0` y `r <= r_next`.
3. Al término de 32 iteraciones (`iter` decrece de 31 a 0), el registro `q` contiene exactamente el valor en **Q16.16**:
   - `q[31:16]`: Parte entera de $\sqrt{n}$
   - `q[15:0]`: Parte fraccionaria de $\sqrt{n}$

---

### 4.4 FSM del Receptor UART (`uart_rx.v`)

El módulo [`uart_rx`](file:///Users/Azu/Desktop/School/UDLAP/9o/Sistemas%20Embebidos/Parcial3/Practica4/uart_rx.v) implementa un receptor asíncrono con sobremuestreo y validación en el centro del bit de inicio. Consta de una FSM de **4 estados** (registro `state` de 2 bits).

#### Diagrama de Estados:

```mermaid
stateDiagram-v2
    [*] --> S_IDLE
    
    S_IDLE --> S_START_BIT: rx == 0 (Detección de flanco de bajada)
    S_IDLE --> S_IDLE: rx == 1
    
    S_START_BIT --> S_DATA_BITS: clk_count == HALF_BIT - 1 && rx == 0\n(Start bit validado en t = 0.5 Tbit)
    S_START_BIT --> S_IDLE: clk_count == HALF_BIT - 1 && rx == 1\n(Falso start bit / Glitch)
    S_START_BIT --> S_START_BIT: clk_count < HALF_BIT - 1\n[clk_count <= clk_count + 1]
    
    S_DATA_BITS --> S_DATA_BITS: clk_count < CLKS_PER_BIT - 1\n[clk_count <= clk_count + 1]
    S_DATA_BITS --> S_DATA_BITS: clk_count == CLKS_PER_BIT - 1 && bit_index < 7\n[rx_shift[bit_index] <= rx, bit_index <= bit_index + 1, clk_count <= 0]
    S_DATA_BITS --> S_STOP_BIT: clk_count == CLKS_PER_BIT - 1 && bit_index == 7\n[rx_shift[7] <= rx, bit_index <= 0, clk_count <= 0]
    
    S_STOP_BIT --> S_STOP_BIT: clk_count < CLKS_PER_BIT - 1\n[clk_count <= clk_count + 1]
    S_STOP_BIT --> S_IDLE: clk_count == CLKS_PER_BIT - 1\n[data_out <= rx_shift, data_valid <= 1, clk_count <= 0]
```

#### Descripción de Estados:

| Estado | Código | Función / Comportamiento |
|---|---|---|
| `S_IDLE` | `2'b00` | Línea en reposo (`rx == 1`). Al detectar flanco de bajada (`rx == 0`), pasa a `S_START_BIT`. |
| `S_START_BIT` | `2'b01` | Cuenta hasta el centro del bit (`HALF_BIT = 2604` ciclos a 50 MHz). Si la línea permanece en bajo, confirma el bit de inicio y pasa a `S_DATA_BITS`; de lo contrario, se descarta por ruido y regresa a `S_IDLE`. |
| `S_DATA_BITS` | `2'b10` | Muestrea secuencialmente cada uno de los 8 bits en el centro de su ventana (`CLKS_PER_BIT = 5208` ciclos), desplazándolos en `rx_shift` de LSB a MSB. |
| `S_STOP_BIT` | `2'b11` | Aguarda el tiempo del bit de parada, asigna `data_out <= rx_shift`, pulsa `data_valid <= 1` durante un ciclo de reloj y regresa a `S_IDLE`. |

---

### 4.5 Máquina de Control del Transmisor UART (`uart_tx.v`)

El módulo [`uart_tx`](file:///Users/Azu/Desktop/School/UDLAP/9o/Sistemas%20Embebidos/Parcial3/Practica4/uart_tx.v) serializa bytes hacia el pin físico `UART_TX`. Opera mediante un autómata de **2 modos de operación** gobernado por el registro `busy` y un registro de desplazamiento de 10 bits (`tx_shift`).

#### Diagrama de Estados:

```mermaid
stateDiagram-v2
    [*] --> STATE_IDLE
    
    STATE_IDLE --> STATE_TRANSMIT: tx_start == 1\n[busy <= 1, tx_shift <= {1'b1, data_in, 1'b0}, clk_count <= 0, bit_index <= 0]
    STATE_IDLE --> STATE_IDLE: tx_start == 0\n[tx <= 1]
    
    STATE_TRANSMIT --> STATE_TRANSMIT: clk_count < CLKS_PER_BIT - 1\n[clk_count <= clk_count + 1, tx <= tx_shift[0]]
    STATE_TRANSMIT --> STATE_TRANSMIT: clk_count == CLKS_PER_BIT - 1 && bit_index < 9\n[tx_shift <= {1'b1, tx_shift[9:1]}, bit_index <= bit_index + 1, clk_count <= 0]
    STATE_TRANSMIT --> STATE_IDLE: clk_count == CLKS_PER_BIT - 1 && bit_index == 9\n[busy <= 0, tx <= 1]
```

#### Funcionamiento:
- **Reposo (`busy == 0`):** La línea `tx` se mantiene en alto lógico (`1`). Al recibir el pulso `tx_start`, se carga en el registro de 10 bits la trama serial completa `{1'b1, data_in, 1'b0}` (Bit de parada `1`, 8 bits de datos, Bit de inicio `0`), se activa `busy <= 1` y se limpian contadores.
- **Transmisión (`busy == 1`):** La línea `tx` expone `tx_shift[0]`. Cada `CLKS_PER_BIT` ciclos (5208 ciclos a 9600 baudios), el registro se desplaza a la derecha insertando un `1` en el MSB. Tras 10 desplazamientos (`bit_index == 9`), se desactiva `busy <= 0` y la línea regresa al reposo.

---

## 5. Estructura de Archivos del Proyecto

| Archivo | Descripción | Contiene FSM |
|---|---|:---:|
| [`top_uart_de10lite.v`](file:///Users/Azu/Desktop/School/UDLAP/9o/Sistemas%20Embebidos/Parcial3/Practica4/top_uart_de10lite.v) | Módulo Top-Level (Controlador maestro, FSM de 8 estados, UART y Coprocesador) | **Sí (8 estados)** |
| [`process_number.v`](file:///Users/Azu/Desktop/School/UDLAP/9o/Sistemas%20Embebidos/Parcial3/Practica4/process_number.v) | FSM del Coprocesador Parametrizado para $n^{p/2}$ con cálculo concurrente | **Sí (4 estados)** |
| [`square_root.v`](file:///Users/Azu/Desktop/School/UDLAP/9o/Sistemas%20Embebidos/Parcial3/Practica4/square_root.v) | FSM de Raíz Cuadrada no restaurativa dígito a dígito en Q16.16 (32 iteraciones) | **Sí (3 estados)** |
| [`uart_rx.v`](file:///Users/Azu/Desktop/School/UDLAP/9o/Sistemas%20Embebidos/Parcial3/Practica4/uart_rx.v) | FSM del Receptor UART con validación de Start Bit en centro de bit | **Sí (4 estados)** |
| [`uart_tx.v`](file:///Users/Azu/Desktop/School/UDLAP/9o/Sistemas%20Embebidos/Parcial3/Practica4/uart_tx.v) | Autómata de Transmisión UART con registro de desplazamiento de 10 bits | **Sí (2 modos)** |
| [`tb_top.v`](file:///Users/Azu/Desktop/School/UDLAP/9o/Sistemas%20Embebidos/Parcial3/Practica4/tb_top.v) | Banco de pruebas completo para ModelSim / QuestaSim con tareas para $n$ y $p$ | No (Testbench) |
| [`arduino.cpp`](file:///Users/Azu/Desktop/School/UDLAP/9o/Sistemas%20Embebidos/Parcial3/Practica4/arduino.cpp) | Firmware del ESP32: Pasarela transparente USB $\leftrightarrow$ UART con LED diagnóstico | No (Firmware) |
| [`test_uart_fpga.m`](file:///Users/Azu/Desktop/School/UDLAP/9o/Sistemas%20Embebidos/Parcial3/Practica4/test_uart_fpga.m) | Consola interactiva en MATLAB para envío de $n$ y $p$ con cálculo de error | No (Script) |
| [`README.md`](file:///Users/Azu/Desktop/School/UDLAP/9o/Sistemas%20Embebidos/Parcial3/Practica4/README.md) | Documentación técnica integral del proyecto | No |

---

## 6. Verificación, Simulación y Pruebas

### 6.1 Simulación RTL en ModelSim / QuestaSim (`tb_top.v`)
El banco de pruebas [`tb_top.v`](file:///Users/Azu/Desktop/School/UDLAP/9o/Sistemas%20Embebidos/Parcial3/Practica4/tb_top.v) incluye la tarea `send_uart_frame_np(n_val, p_val)` para enviar tramas con base $n$ y exponente $p$, así como monitoreo en tiempo real de las transiciones de la FSM de la FPGA:

```bash
# Compilar todos los módulos Verilog
vlog top_uart_de10lite.v process_number.v square_root.v uart_rx.v uart_tx.v tb_top.v

# Ejecutar simulación
vsim -c tb_top_uart_de10lite -do "run -all; quit"
```

### 6.2 Casos de Prueba Típicos ($n^{p/2}$)

| Base $n$ | Exponente $p$ | Expresión Evaluada | Valor Teórico | Hexadecimal FPGA (Q80.16) | Parte Entera | Parte Fraccionaria | Resultado FPGA | Error Absoluto |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **10** | **5** | $10^2 \cdot \sqrt{10}$ | **316.227766** | `0x000000000000013C3A4F` | 316 | 14927 / 65536 | **316.227783** | $1.7 \times 10^{-5}$ |
| **50** | **5** | $50^2 \cdot \sqrt{50}$ | **17677.669530** | `0x000000000000450DAB65` | 17677 | 43877 / 65536 | **17677.669510** | $2.0 \times 10^{-5}$ |
| **85** | **5** | $85^2 \cdot \sqrt{85}$ | **66610.665400** | `0x0000000000010432AA5A` | 66610 | 43610 / 65536 | **66610.665390** | $1.0 \times 10^{-5}$ |
| **144**| **5** | $144^2 \cdot 12$ | **248832.000000** | `0x000000000003CC000000` | 248832 | 0 / 65536 | **248832.000000** | **0.000000** |
| **4**  | **4** | $4^{4/2} = 4^2$ | **16.000000** | `0x00000000000000100000` | 16 | 0 / 65536 | **16.000000** | **0.000000** |
| **9**  | **3** | $9^1 \cdot \sqrt{9} = 27$ | **27.000000** | `0x000000000000001B0000` | 27 | 0 / 65536 | **27.000000** | **0.000000** |

> [!NOTE]
> Cuando el exponente $p$ es par o cuando la base $n$ es un cuadrado perfecto, el componente fraccionario es exactamente cero (`0x0000`), demostrando error cero. En casos con raíz irracional, el error absoluto se mantiene estrictamente por debajo de la resolución fraccionaria de 16 bits ($\frac{1}{65536} \approx 1.5258 \times 10^{-5}$).

---

### 6.3 Firmware de la Pasarela ESP32 (`arduino.cpp`)

El microcontrolador ESP32 actúa como un puente bidireccional transparente:
- **`Serial` (USB):** Conecta a la PC a 9600 baudios. Recibe la trama de 7 bytes desde MATLAB y reenvía los 14 bytes devueltos por la FPGA.
- **`Serial2` (Hardware UART):** Conecta directamente con la FPGA a 9600 baudios (8N1):
  - **Pin GPIO 16 (`RXD2`):** Conectar a `UART_TX` de la FPGA.
  - **Pin GPIO 17 (`TXD2`):** Conectar a `UART_RX` de la FPGA.
  - **GND:** Referencia común obligatoria entre el ESP32 y la placa DE10-Lite.
- **LED Diagnóstico (`LED_PIN = 2`):** Se enciende automáticamente al remitir la trama a la FPGA y permanece encendido mientras aguarda los 14 bytes de respuesta, apagándose al completar la transacción.

---

### 6.4 Consola Interactiva en MATLAB (`test_uart_fpga.m`)

El script interactivo [`test_uart_fpga.m`](file:///Users/Azu/Desktop/School/UDLAP/9o/Sistemas%20Embebidos/Parcial3/Practica4/test_uart_fpga.m) permite evaluar el hardware en tiempo real:
1. Abre la conexión serial en el puerto configurado (ej. `COM8` en Windows o `/dev/ttyUSB0` en Linux/macOS) a 9600 baudios.
2. Solicita interactivamente la base $n \ge 0$ y el exponente $p \ge 0$ (o un valor negativo para salir).
3. Construye y transmite la trama de 7 bytes `[0xAA, n[7:0], n[15:8], n[23:16], n[31:24], p[7:0], 0x55]`.
4. Implementa una máquina de recepción con sincronización por cabecera `0xAA`, descarte de basura previa y timeout de 1.5 s.
5. Valida el delimitador final `0x55`, desempaqueta los 12 bytes del resultado Q80.16 y calcula el error absoluto respecto al cálculo teórico `double(n) ^ (double(p) / 2.0)`.

---

## 7. Mapeo de Pines en la Tarjeta Terasic DE10-Lite

Para la síntesis en Intel Quartus Prime, se utilizan las siguientes asignaciones de pines en el archivo de restricciones (`.qsf`):

| Puerto Verilog | Tipo | Pin FPGA MAX 10 | Descripción en Placa DE10-Lite |
|---|---|---|---|
| `CLOCK_50` | Input (3.3V LVTTL) | `PIN_P11` | Oscilador de reloj de 50 MHz del sistema |
| `RESET_N` | Input (3.3V LVTTL) | `PIN_B8` | Pulsador activo en bajo `KEY[0]` |
| `UART_RX` | Input (3.3V LVTTL) | `PIN_V10` | GPIO [0] |
| `UART_TX` | Output (3.3V LVTTL) | `PIN_W10` | GPIO [1] |

> [!IMPORTANT]
> Los pines del conector de expansión GPIO y de la cabecera Arduino en la tarjeta DE10-Lite operan a **3.3V LVTTL**. Dado que el ESP32 opera de forma nativa a 3.3V en sus líneas lógicas, la conexión directa de los pines TX/RX entre ambas tarjetas es totalmente segura.
