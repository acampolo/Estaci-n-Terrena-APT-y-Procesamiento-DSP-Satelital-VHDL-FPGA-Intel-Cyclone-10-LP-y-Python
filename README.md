# Estación Terrena APT y Procesamiento DSP Satelital - VHDL - FPGA - Intel Cyclone 10 LP y Python
![Hardware](https://img.shields.io/badge/Hardware-Intel_Cyclone_10_LP-blue?style=for-the-badge&logo=intel)
![VHDL](https://img.shields.io/badge/HDL-VHDL-orange?style=for-the-badge)
![Python](https://img.shields.io/badge/Software-Python_3.10+-yellow?style=for-the-badge&logo=python)
![OpenCV](https://img.shields.io/badge/Vision-OpenCV-green?style=for-the-badge&logo=opencv)
![PyQt6](https://img.shields.io/badge/GUI-PyQt6-lightgrey?style=for-the-badge)

> **Sistema embebido de teledetección satelital de grado instrumental basado en tecnología FPGA, demodulación analógica de amplitud y frecuencia, y procesamiento de imágenes asíncrono en Python.**

Este proyecto documenta el diseño, la construcción y la validación de una estación terrena regida por la directriz mecatrónica VDI 2206. A diferencia de las plataformas SDR tradicionales que sufren cuellos de botella debido a la latencia propia del sistema operativo, este ecosistema traslada el cómputo DSP pesado, es decir, la extracción de envolvente y los filtros FIR, directamente al silicio reconfigurable de una matriz FPGA. De este modo, la PC se libera de esa carga computacional y asume únicamente las tareas asíncronas de renderizado visual, además del filtrado espacial apoyado en algoritmos de visión artificial.

---

## 📋 Tabla de Contenidos
1. [Características Principales](#-características-principales)
2. [Arquitectura del Sistema](#-arquitectura-del-sistema)
3. [Diseño de Hardware y Radiofrecuencia](#-diseño-de-hardware-y-radiofrecuencia)
4. [Firmware DSP en FPGA - VHDL](#-firmware-dsp-en-fpga---vhdl)
5. [Software de Estación Terrena - Python](#-software-de-estación-terrena---python)
6. [Resultados y Consumo de Recursos](#-resultados-y-consumo-de-recursos)
7. [Autores](#-autores)

---

## 🚀 Características Principales

- **RF VHF Estabilizado**: Implementa un acondicionamiento analógico activo junto con una conmutación automática de transmisión conocida como PTT para transceptores SA828 sintonizados a 145.5 MHz.
- **Demodulación AM en Silicio**: Desarrolla la Transformada de Hilbert estructurada como un filtro FIR en lenguaje VHDL, de forma que la extracción de envolvente emplea la aproximación matemática $\alpha$-max $\beta$-min para lograr una latencia casi nula.
- **Sincronización por Contraste Dinámico**: Incorpora un algoritmo a nivel de hardware que es totalmente inmune al desvanecimiento de señal, conocido en telecomunicaciones como fading, así como al ruido impulsivo, operando a una tasa constante de 48 kHz.
- **Telemetría I2C Concurrente**: Ejecuta una fusión de los datos provenientes del barómetro MS5611, el acelerómetro ADXL345 y el sensor de clima SHT31, mostrando toda esta instrumentación localmente a través de una pantalla OLED SSD1306.
- **Multiplexación UART a Alta Velocidad**: Cuenta con un transmisor asíncrono configurado a 921600 baudios que se encarga de despachar paquetes híbridos, los cuales están compuestos por video e instrumentación, empleando para ello una memoria FIFO asíncrona que evita cualquier colisión de datos.
- **Visión Artificial Híbrida**: Dispone de una interfaz gráfica multi-hilo programada en PyQt6, gracias a la cual la matriz de 256x256 px que fluye a 2400 px/s se purifica en tiempo real usando un filtrado bilateral y una ecualización adaptativa CLAHE mediante la librería OpenCV.

---

## 📡 Arquitectura del Sistema

El flujo de datos obedece un esquema de codiseño estricto entre el hardware y el software. Por un lado, el formato de la trama emula el estándar APT clásico pero se encuentra adaptado para microcontroladores, por lo que fija la subportadora acústica en 1800 Hz y la frecuencia de muestreo en 48 kHz.

| Subsistema Lógico | Dominio de Reloj | Protocolo y Tasa | Función Asignada |
| :--- | :--- | :--- | :--- |
| **Receptor I2S** | Reloj interno de 50 MHz sincronizado | 48 kHz a 24 bits | Captura acústica desde el ADC Delta-Sigma PCM1808. |
| **Maestros I2C** | Reloj interno de 50 MHz | 400 kHz en Fast Mode | Recolección de variables inerciales y despliegue local en la pantalla OLED. |
| **Transmisor UART** | Reloj interno de 50 MHz | 921600 baudios | Evacuación multiplexada asíncrona de datos hacia la computadora. |
| **Visión OpenCV** | Escritorio procesado en la CPU Host | Refresco visual cada 50 ms | Filtrado fotográfico asíncrono de alta definición gestionado mediante hilos. |

---

## 🛠️ Diseño de Hardware y Radiofrecuencia

El acondicionamiento físico de la señal eléctrica resulta un paso crítico para garantizar un espectro analógico limpio antes de que intervenga la digitalización de los datos.

### 1. Acondicionamiento Analógico
- **Filtros Activos MCP6022:** Se seleccionaron estos amplificadores operacionales gracias a su gran ancho de banda de 10 MHz y a su bajo nivel de ruido estimado en 8.7 nV/√Hz. En la placa receptora, un arreglo paso banda en cascada acotado entre 106 Hz y 7234 Hz resguarda al ADC. En contraparte, en la placa emisora, un filtro paso altas se encarga de bloquear la corriente continua y proporciona una ganancia calculada de 3.2.
- **Tierra Virtual:** Un divisor resistivo totalmente simétrico establece un voltaje de referencia de 1.65 V, de modo que la onda de audio de corriente alterna puede oscilar de manera segura sin rebasar el límite de alimentación establecido en 3.3 V.
- **Conmutación Automática:** El sistema cuenta con un diodo Schottky SS14 que rectifica la señal de audio y procede a cargar un circuito pasivo cuya constante de tiempo es de 220.9 ms. Cuando este voltaje excede el umbral de disparo, se satura un transistor BC547, el cual lleva a tierra el pin de transmisión encendiendo la etapa de radiofrecuencia en tan solo 3.8 ms.

### 2. Antenas y Transceptores
- **Módulos SA828:** Estos componentes operan en la banda de radioaficionados de 2 metros, sintonizados específicamente en los 145.5 MHz. Cabe mencionar que los filtros de voz predeterminados y los tonos subaudibles CTCSS fueron desactivados mediante la herramienta de configuración NiceRF para así obtener un canal de audio completamente transparente a la modulación.
- **Arreglo de Antenas:** Se instaló una antena monopolo omnidireccional de cuarto de onda para la emisión de la señal, mientras que para la recepción se utilizó un dipolo altamente direccional dispuesto en V con una apertura de 120 grados.

> ⚠️ **Correcciones Físicas de Diseño tras el Ensamblaje:**
> Durante el proceso de integración, descubrimos empíricamente que el pin de salida de potencia SPK del módulo SA828 estaba saturando catastróficamente los amplificadores debido a que entregaba hasta 2 W de fuerza. Para solucionar este inconveniente, realizamos un corte mecánico en la pista de cobre y creamos un puente físico dirigido hacia el pin VOL, el cual provee un nivel de línea adecuado. Paralelamente, agregamos una resistencia de montaje superficial tipo pull-down en el capacitor del detector de señal, con lo cual garantizamos que la energía se descargue velozmente al dar por finalizada la transmisión.

---

## 🧠 Firmware DSP en FPGA - VHDL

La estrategia para extraer la subportadora prescinde de los tradicionales osciladores controlados numéricamente, conocidos en la literatura técnica como NCO. En cambio, desarrolla un robusto cauce de procesamiento estructurado en 7 etapas y basado en la **Transformada de Hilbert**.

### Motor Matemático: Aproximación $\alpha$-max $\beta$-min
Con el objetivo de evitar el desbordamiento numérico, y para no acaparar los bloques lógicos de la FPGA tratando de calcular raíces cuadradas complejas, la magnitud geométrica del vector bidimensional se obtiene empleando simples desplazamientos lógicos de bits.

```vhdl
-- ETAPA 5: Ordenamiento Lógico en Máximo y Mínimo
if valid_pipe(4) = '1' then
    if abs_I > abs_Q then
        max_val <= abs_I; min_val <= abs_Q;
    else
        max_val <= abs_Q; min_val <= abs_I;
    end if;
end if;

-- ETAPA 6: Aproximación de la Magnitud calculada como el Máximo más tres octavos
-- del Mínimo, empleando únicamente sumas y desplazamientos de registros
if valid_pipe(5) = '1' then
    mag_approx <= resize(max_val, 27) +
                  shift_right(resize(min_val, 27), 2) +
                  shift_right(resize(min_val, 27), 3);
end if;
```

De esta manera, el filtro digital FIR de 15 coeficientes saca partido de los bloques multiplicadores DSP físicos alojados dentro de la FPGA. Con esto se asegura una latencia fija e inamovible de 145.8 µs, aportando a su vez un alto grado de respuesta determinista.

### Multiplexor UART
La telemetría recolectada por el bus I2C y los píxeles de la matriz de video confluyen directamente en el módulo multiplexor de paquetes. Una máquina de estados finitos es la responsable de arbitrar este intenso tráfico de red. Su funcionamiento radica en aprovechar los silencios acústicos correspondientes a los porches negros de la imagen para interrumpir momentáneamente el flujo de video, despachar en su lugar los bytes de aceleración y temperatura encapsulados con la cabecera hexadecimal fija compuesta por `0xA5` y `0x5A`, y finalmente retomar el envío de la fotografía.

---

## 💻 Software de Estación Terrena - Python

El programa receptor se encuentra codificado en lenguaje Python, valiéndose del entorno de ventanas PyQt6. Para prevenir que el aluvión de datos a 921600 baudios bloquee la interfaz de usuario, se aisló la lectura continua del puerto serie derivándola a un hilo de trabajo en segundo plano mediante la clase `QThread`.

### Pipeline de Visión Artificial con OpenCV
El ruido impulsivo que subsiste a la recepción analógica del canal es mitigado aplicando un enfoque de procesamiento bidimensional que protege celosamente la topografía de la fotografía captada.

```python
def processed_frame(self):
    img = self.frame.copy()

    if self.chk_median.isChecked():
        # 1. Filtro Bilateral: suaviza la estática acumulada en el fondo a la vez que
        # preserva la nitidez de los bordes finos de los continentes o la nubosidad.
        suavizado = cv2.bilateralFilter(img, d=5, sigmaColor=50, sigmaSpace=50)

        # 2. Ecualización adaptativa de contraste CLAHE:
        # ajusta los niveles de iluminación sobre cuadrículas de 8x8 píxeles
        # para revelar detalles ocultos a nivel local.
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        self.filtered = clahe.apply(suavizado)
        img = self.filtered
    else:
        # Sin filtros de visión artificial, se aplica un ajuste de contraste
        # global mediante el cálculo de percentiles.
        lo, hi = np.percentile(img, [2, 98])
        if hi > lo:
            img = np.clip((img.astype(np.float32) - lo) * (255.0 / (hi - lo)), 0, 255).astype(np.uint8)

    colored = self.cmap(img / 255.0)
    return np.ascontiguousarray((colored[:, :, :3] * 255).astype(np.uint8))
```

### Fusión Continua de Telemetría
El software de la estación recibe la encomienda de decodificar las ráfagas asíncronas y efectuar cálculos sobre las variables físicas en tiempo real. Como ejemplo de esto, al procesar de manera combinada los datos de presión y de temperatura, el script es capaz de calcular la densidad atmosférica del aire, representada mediante la variable $\rho$. A su vez, monitorea constantemente la lectura de aceleración y guarda un historial continuo para activar alertas preventivas por vibración en el panel de diagnóstico visual.

---

## 📊 Resultados y Consumo de Recursos

La decisión arquitectónica de reescribir desde cero el bloque matemático, esquivando las operaciones de división e instanciando en su lugar sumas de aproximación geométrica, tuvo como resultado directo un consumo de hardware extremadamente reducido en la placa lógica. Este bajo nivel de ocupación garantiza un gran margen operativo en caso de que en un futuro se requiera escalar la resolución fotográfica o añadir bloques de compresión algorítmica.

| Recurso en la FPGA Intel Cyclone 10 LP | Cantidad Física Utilizada | Módulo Principal Responsable de la Ocupación |
| :--- | :--- | :--- |
| **Elementos Lógicos (LE)** | 4765 de 81264 (6%) | Instancias de los controladores I2C junto a la máquina UART y la lógica combinacional central. |
| **Multiplicadores Embebidos DSP de 9 bits** | 30 de 488 (6%) | Filtro matemático de Hilbert dedicado a la convolución apoyada en estos bloques especializados. |
| **Memoria Interna RAM M9K** | 50624 bits en total | Bloques designados para la memoria FIFO asíncrona y la memoria de solo lectura de la pantalla OLED. |
| **Pines de Entrada y Salida (E/S)** | 29 de 290 (10%) | Pines empleados por los relojes de audio, los terminales de sensores y el puerto de transmisión hacia la computadora. |

### Validación Operativa de Laboratorio
- **Análisis de Espectro de Radio con SDR++:** Se comprobó una emisión analógica nítida anclada en los 145.5 MHz, respetando estrictamente el ancho de banda de 12.5 kHz sin generar armónicos destructivos.
- **Resiliencia de Datos frente al Ruido:** Se provocó un bombardeo eléctrico induciendo más de 5000 bytes de basura digital, ante lo cual el despachador lógico programado en la computadora demostró ser capaz de descartar el ruido y resincronizarse velozmente amparándose en la detección de las cabeceras hexadecimales y en la suma de comprobación XOR.
- **Fidelidad y Estabilidad de Geometría Visual:** Las imágenes captadas empleando los patrones de calibración probaron conservar intacta su relación de aspecto original de 256x256 px, logrando evadir totalmente los cortes de fila gracias al alineamiento regido por el contraste dinámico implementado dentro de la FPGA.
