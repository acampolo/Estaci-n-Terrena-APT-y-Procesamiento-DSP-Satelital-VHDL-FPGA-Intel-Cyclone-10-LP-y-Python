# Estación Terrena APT y Procesamiento DSP Satelital - VHDL - FPGA Intel Cyclone 10 LP y Python

![Hardware](https://img.shields.io/badge/Hardware-Intel_Cyclone_10_LP-blue?style=for-the-badge&logo=intel)
![Placa](https://img.shields.io/badge/Placa-StratoSky_FPGA-black?style=for-the-badge)
![VHDL](https://img.shields.io/badge/HDL-VHDL-orange?style=for-the-badge)
![Python](https://img.shields.io/badge/Software-Python_3.10+-yellow?style=for-the-badge&logo=python)
![OpenCV](https://img.shields.io/badge/Vision-OpenCV-green?style=for-the-badge&logo=opencv)
![PyQt6](https://img.shields.io/badge/GUI-PyQt6-lightgrey?style=for-the-badge)

> **Sistema embebido de teledetección satelital basado en una FPGA, con demodulación de amplitud y procesamiento de imágenes en Python.**

En este proyecto diseñé, construí y validé yo solo una estación terrena, siguiendo la guía de diseño mecatrónico VDI 2206. En una plataforma SDR normal, el procesamiento de señal corre en la PC y depende de la latencia del sistema operativo. Aquí hice lo contrario: el trabajo pesado de DSP (la extracción de envolvente y los filtros FIR) lo hace la FPGA. La PC solo se encarga de mostrar la imagen y de aplicarle filtros de visión artificial.

La FPGA que usé es la de la placa **StratoSky FPGA** (Proyecto Estratósfera, Cápsula Electrónica), que lleva una Intel Cyclone 10 LP `10CL080YU484I7G`.

---

## 📋 Tabla de Contenidos
1. [Características Principales](#-características-principales)
2. [Plataforma: Placa StratoSky FPGA](#-plataforma-placa-stratosky-fpga)
3. [Arquitectura del Sistema](#-arquitectura-del-sistema)
4. [Diseño de Hardware y Radiofrecuencia](#-diseño-de-hardware-y-radiofrecuencia)
5. [Firmware DSP en FPGA - VHDL](#-firmware-dsp-en-fpga---vhdl)
6. [Software de Estación Terrena - Python](#-software-de-estación-terrena---python)
7. [Resultados y Consumo de Recursos](#-resultados-y-consumo-de-recursos)
8. [Autor](#-autor)

---

## 🚀 Características Principales

- **RF en VHF**: Armé el acondicionamiento analógico activo y una conmutación automática de transmisión (PTT) para dos módulos SA828 sintonizados a 145.5 MHz.
- **Demodulación AM en la FPGA**: Implementé la Transformada de Hilbert como un filtro FIR en VHDL. La envolvente se calcula con la aproximación $\alpha$-max $\beta$-min, que evita raíces cuadradas y mantiene la latencia muy baja.
- **Sincronización por contraste dinámico**: La sincronización de las líneas se hace dentro de la FPGA y resiste bien el desvanecimiento de la señal (fading) y el ruido impulsivo. Trabaja a una tasa constante de 48 kHz.
- **Telemetría I2C**: Leo el barómetro MS5611, el acelerómetro ADXL345 y el sensor de temperatura y humedad SHT31, y muestro los datos en una pantalla OLED SSD1306.
- **Multiplexación UART**: Un transmisor a 921600 baudios envía paquetes que mezclan video y telemetría. Uso una FIFO asíncrona para que los datos no choquen entre sí.
- **Visión artificial**: La interfaz está hecha en PyQt6 con varios hilos. La imagen de 256x256 px, que llega a 2400 px/s, se limpia en tiempo real con un filtro bilateral y ecualización adaptativa CLAHE de OpenCV.

---

## 🧩 Plataforma: Placa StratoSky FPGA

Todo el firmware corre en la placa StratoSky FPGA. Estas son las partes de la placa que usé o que me sirvieron para el proyecto:

| Elemento | Detalle |
| :--- | :--- |
| **FPGA** | Intel Cyclone 10 LP `10CL080YU484I7G`: 81264 LEs, 2810880 bits de memoria embebida, 488 multiplicadores de 9 bits (244 bloques DSP) y 4 PLLs. |
| **Reloj** | Oscilador externo de 50 MHz (pin G22), que es el reloj principal de todo el diseño. |
| **Programación** | Programador Arrow USB-Blaster integrado, por el puerto USB Tipo-C. No necesité hardware externo. |
| **UART** | A través del mismo USB Tipo-C aparece un puerto COM virtual (pines RX: J3, TX: J4, a 3.3 V). |
| **Sensores en la placa** | SHT31 (I²C, dirección `0x44`), MS5611 (I²C `0x76`/`0x77` o SPI) y ADXL345 (I²C `0x53`/`0x1D` o SPI). También trae un ADC ADS1115 (I²C, `0x48`). |
| **Memoria** | SDRAM Winbond W9825G6KH-6 de 256 Mbit (32 MB) y dos ranuras microSD. |
| **Entradas y salidas** | 10 switches, 10 LEDs y 2 botones con pull-up. |
| **Expansión** | Conectores GPIO1 (1×20), GPIO2 (2×20) y GPIO3 (2×9, pensado para cámara OV7670). |
| **Alimentación** | 5 V por USB Tipo-C o por el header VIN (solo una fuente a la vez). |

Los tres sensores ya vienen soldados en la placa, así que no tuve que cablearlos aparte. Lo que sí conecté desde los conectores de expansión fue el ADC de audio PCM1808 y la pantalla OLED.

---

## 📡 Arquitectura del Sistema

La trama imita el formato APT clásico, pero adaptado a este sistema: la subportadora acústica está en 1800 Hz y la frecuencia de muestreo es de 48 kHz.

| Subsistema | Dominio de Reloj | Protocolo y Tasa | Función |
| :--- | :--- | :--- | :--- |
| **Receptor I2S** | Reloj interno de 50 MHz | 48 kHz a 24 bits | Captura el audio desde el ADC Delta-Sigma PCM1808. |
| **Maestros I2C** | Reloj interno de 50 MHz | 400 kHz (Fast Mode) | Lee los sensores y muestra los datos en la pantalla OLED. |
| **Transmisor UART** | Reloj interno de 50 MHz | 921600 baudios | Envía los datos multiplexados a la computadora. |
| **Visión OpenCV** | CPU de la PC | Refresco cada 50 ms | Filtra la imagen en la PC, con hilos separados. |

---

## 🛠️ Diseño de Hardware y Radiofrecuencia

Antes de digitalizar, la señal analógica tiene que llegar lo más limpia posible, así que dediqué bastante tiempo al acondicionamiento.

### 1. Acondicionamiento Analógico
- **Filtros activos con MCP6022:** Elegí estos amplificadores operacionales por su ancho de banda de 10 MHz y su bajo ruido (8.7 nV/√Hz). En la placa receptora puse un filtro paso banda en cascada entre 106 Hz y 7234 Hz para proteger el ADC. En la placa emisora usé un filtro paso altas que bloquea la corriente continua y da una ganancia de 3.2.
- **Tierra virtual:** Un divisor resistivo simétrico genera una referencia de 1.65 V. Así la señal de audio puede oscilar sin pasarse de la alimentación de 3.3 V.
- **Conmutación automática:** Un diodo Schottky SS14 rectifica la señal de audio y carga un circuito pasivo con una constante de tiempo de 220.9 ms. Cuando el voltaje supera el umbral, un transistor BC547 se satura y lleva a tierra el pin de transmisión, lo que enciende la etapa de RF en unos 3.8 ms.

### 2. Antenas y Transceptores
- **Módulos SA828:** Trabajan en la banda de radioaficionados de 2 metros, sintonizados a 145.5 MHz. Con la herramienta de configuración de NiceRF desactivé los filtros de voz y los tonos subaudibles CTCSS, para que el canal de audio no altere la modulación.
- **Antenas:** Para transmitir usé un monopolo omnidireccional de cuarto de onda. Para recibir usé un dipolo direccional en V con una apertura de 120 grados.

> ⚠️ **Correcciones de diseño después del ensamblaje:**
> Al integrar todo descubrí que la salida de potencia SPK del módulo SA828 saturaba los amplificadores, porque entregaba hasta 2 W. Corté la pista de cobre y la reemplacé con un puente hacia el pin VOL, que entrega un nivel de línea adecuado. Además, agregué una resistencia SMD pull-down en el capacitor del detector de señal, para que se descargue rápido cuando termina la transmisión.

---

## 🧠 Firmware DSP en FPGA - VHDL

Para extraer la subportadora no usé un oscilador controlado numéricamente (NCO). En su lugar armé un pipeline de 7 etapas basado en la **Transformada de Hilbert**.

### Cálculo de magnitud: aproximación $\alpha$-max $\beta$-min
Para evitar desbordamientos y no gastar lógica de la FPGA en raíces cuadradas, calculo la magnitud del vector con sumas y desplazamientos de bits.

```vhdl
-- ETAPA 5: Ordenar en máximo y mínimo
if valid_pipe(4) = '1' then
    if abs_I > abs_Q then
        max_val <= abs_I; min_val <= abs_Q;
    else
        max_val <= abs_Q; min_val <= abs_I;
    end if;
end if;

-- ETAPA 6: Magnitud ≈ máximo + 3/8 del mínimo,
-- usando solo sumas y desplazamientos de registros
if valid_pipe(5) = '1' then
    mag_approx <= resize(max_val, 27) +
                  shift_right(resize(min_val, 27), 2) +
                  shift_right(resize(min_val, 27), 3);
end if;
```

El filtro FIR de 15 coeficientes usa los multiplicadores DSP que ya trae la FPGA. La latencia es fija, de 145.8 µs, y la respuesta es determinista.

### Multiplexor UART
La telemetría de los sensores I2C y los píxeles de la imagen llegan al mismo módulo multiplexor, que controlo con una máquina de estados finitos. Aprovecho los porches negros de la imagen, que son los momentos sin señal útil, para pausar el video, enviar los bytes de aceleración y temperatura con una cabecera fija (`0xA5` y `0x5A`) y después seguir con la imagen.

---

## 💻 Software de Estación Terrena - Python

El programa receptor está hecho en Python con PyQt6. Como a 921600 baudios llegan muchos datos y podrían congelar la interfaz, la lectura del puerto serie corre en un hilo aparte con `QThread`.

### Procesamiento de imagen con OpenCV
El ruido impulsivo que queda después de la recepción analógica lo reduzco con un procesamiento en dos dimensiones que trata de conservar los detalles de la imagen.

```python
def processed_frame(self):
    img = self.frame.copy()

    if self.chk_median.isChecked():
        # 1. Filtro bilateral: suaviza la estática del fondo
        # y conserva los bordes de continentes y nubes.
        suavizado = cv2.bilateralFilter(img, d=5, sigmaColor=50, sigmaSpace=50)

        # 2. CLAHE: ecualización adaptativa de contraste
        # sobre cuadrículas de 8x8 píxeles, para resaltar detalles locales.
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        self.filtered = clahe.apply(suavizado)
        img = self.filtered
    else:
        # Sin los filtros, ajusto el contraste global con percentiles.
        lo, hi = np.percentile(img, [2, 98])
        if hi > lo:
            img = np.clip((img.astype(np.float32) - lo) * (255.0 / (hi - lo)), 0, 255).astype(np.uint8)

    colored = self.cmap(img / 255.0)
    return np.ascontiguousarray((colored[:, :, :3] * 255).astype(np.uint8))
```

### Telemetría en tiempo real
El software decodifica los paquetes que llegan por el puerto serie y calcula variables físicas al momento. Por ejemplo, con la presión y la temperatura calcula la densidad del aire ($\rho$). También guarda un historial de la aceleración y muestra una alerta de vibración en el panel de diagnóstico.

---

## 📊 Resultados y Consumo de Recursos

Reescribí el bloque matemático desde cero y evité las divisiones, usando sumas de aproximación. Con eso la FPGA de la placa StratoSky queda con mucho espacio libre, que sirve si más adelante quiero subir la resolución de la imagen o agregar compresión.

| Recurso en la FPGA (10CL080YU484I7G) | Utilizado | Dónde se usa |
| :--- | :--- | :--- |
| **Elementos lógicos (LE)** | 4765 de 81264 (6%) | Controladores I2C, máquina UART y lógica central. |
| **Multiplicadores DSP de 9 bits** | 30 de 488 (6%) | Filtro de Hilbert (convolución). |
| **Memoria embebida (M9K)** | 50624 de 2810880 bits (2%) | FIFO asíncrona y memoria de solo lectura de la pantalla OLED. |
| **Pines de E/S** | 29 de 290 (10%) | Relojes de audio, sensores y puerto hacia la computadora. |

### Pruebas de laboratorio
- **Espectro con SDR++:** La emisión se ve limpia en 145.5 MHz, dentro del ancho de banda de 12.5 kHz y sin armónicos importantes.
- **Resistencia al ruido:** Inyecté más de 5000 bytes de basura digital. El programa de la PC descartó el ruido y se resincronizó rápido usando las cabeceras hexadecimales y la suma de comprobación XOR.
- **Geometría de la imagen:** Con patrones de calibración, las imágenes mantuvieron su tamaño de 256x256 px y no hubo cortes de fila, gracias a la sincronización por contraste dinámico implementada en la FPGA.

---

## 👤 Autor

- **Favio Alexander Campolo Ruiz** - Estudiante de Ingeniería Electrónica, Universidad Tecnológica del Perú.