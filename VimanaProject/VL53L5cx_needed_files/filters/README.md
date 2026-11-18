# VL53L1X — Detección de camino libre (ESP8266 / ESP32)

Firmware para el sensor de distancia láser **VL53L1X** que decide si hay un obstáculo enfrente o si el camino está libre, con filtrado robusto contra las lecturas basura del sensor.

> **El problema:** cuando no hay nada enfrente, el VL53L1X casi nunca devuelve una medición válida. La distancia que reporta en ese caso es ruido aleatorio (41–5057 mm). Si usas la distancia cruda, el robot "ve" obstáculos que no existen.
>
> **La solución:** usar el *estado de validez* como señal principal, no la distancia. Las lecturas inválidas se convierten en "sin objetivo" y luego un filtro de mediana + EMA + histéresis estabiliza la decisión.

---

## Tabla de contenido

- [Características](#características)
- [Hardware](#hardware)
- [Cómo funciona](#cómo-funciona)
- [Configuración](#configuración)
- [Salidas](#salidas)

---

## Características

- **Detección de camino libre** basada en validez, no en distancia cruda.
- **Filtro de mediana** (ventana de 5) que actúa como votación por mayoría.
- **EMA con snap** para suavizar sin introducir retardo ante cambios reales.
- **Doble histéresis**: por distancia (1000/1300 mm) y por frames consecutivos (2/4).
- **Dos formatos de salida**: Serial Plotter para depurar en vivo, CSV para análisis posterior.
- Compatible con **ESP8266** (NodeMCU, Wemos D1) y **ESP32**.
- Configuración por `#define` — sin tocar la lógica para ajustar el comportamiento.

---

## Hardware

### Componentes

| Componente | Notas |
|---|---|
| VL53L1X (módulo breakout) | 3.3 V **solamente**. No tolera 5 V. |
| ESP8266 o ESP32 | Cualquier placa con I2C. |
| Pull-ups I2C | 4.7 kΩ a 3.3 V en SDA y SCL si el módulo no los trae. |
| Condensador 100 nF | Opcional, entre VDD y GND cerca del sensor, mejora la estabilidad. |

### Conexiones

| Señal | ESP8266 | ESP32 |
|---|---|---|
| SDA | GPIO4 (D2) | GPIO21 |
| SCL | GPIO5 (D1) | GPIO22 |
| VDD | 3.3 V | 3.3 V |
| GND | GND | GND |

---


## Cómo funciona

El flujo de una muestra es este:

### 1. Validación

Una lectura se acepta solo si cumple las tres condiciones:

- **Estado** = `RangeValid` (0 en la API de Pololu; equivale al raw `9` del sensor).
- **Distancia** entre 20 y 4000 mm.
- **Señal** por encima de `MIN_SENAL_MCPS`.

Opcionalmente se aceptan los estados "débiles" (3, 6, 11) con `ACEPTAR_ESTADOS_DEBILES`.

### 2. Mediana como votación

Las lecturas inválidas se sustituyen por `MAX_RANGE_MM` (4000). Con una ventana de 5, si **3 o más** muestras fueron inválidas, la mediana es 4000 → camino libre.

Un obstáculo aislado no mueve el resultado. Es, literalmente, una votación por mayoría.

### 3. EMA con snap

La media móvil exponencial suaviza el ruido: EMA_n = α · mediana_n + (1 − α) · EMA_(n−1)

Con `α = 0.40`, cada muestra pesa 40%. Si la mediana se aleja más de `EMA_SNAP_MM` (600 mm) de la EMA, se salta directo: un cambio real se detecta sin el retardo del suavizado.

### 4. Doble histéresis

**Por distancia:** para pasar a obstáculo hay que bajar de 1000 mm; para volver a libre hay que superar 1300 mm. La banda intermedia es zona muerta.

**Por tiempo:** se necesitan 2 frames consecutivos para declarar obstáculo y 4 para declarar libre. Un falso "libre" es más peligroso que un falso "obstruido", por eso el segundo es más lento.

---

## Configuración

Todos los parámetros están al inicio del archivo, en la sección `CONFIGURACIÓN`.

### Sensor

| Macro | Valor por defecto | Descripción |
|---|---|---|
| `MODO_LARGO` | `true` | `true` = Long (~4 m), `false` = Short (~1.3 m, mejor con luz de sol). |
| `TIMING_BUDGET_US` | `100000` | Tiempo de integración por medición. Más tiempo = menos ruido. |
| `PERIODO_MS` | `100` | Periodo entre mediciones. Debe ser ≥ timing budget. |
| `ROI_LADO` | `16` | Tamaño de la región de interés. 16 = cono completo (~27°). Mínimo 4. |

### Validación

| Macro | Valor por defecto | Descripción |
|---|---|---|
| `MAX_RANGE_MM` | `4000` | Techo de rango y valor de "camino libre". |
| `MIN_RANGE_MM` | `20` | Piso de rango. |
| `ACEPTAR_ESTADOS_DEBILES` | `false` | Acepta estados 3, 6 y 11 (menos confiables). |
| `MIN_SENAL_MCPS` | `0.0` | Señal mínima del láser. Sube a 0.5–2.0 si entra ruido. |

### Filtro

| Macro | Valor por defecto | Descripción |
|---|---|---|
| `VENTANA_MEDIANA` | `5` | Impar: 3, 5, 7. A 10 Hz, 5 muestras = 0.5 s de memoria. |
| `EMA_ALPHA` | `0.40` | Peso de la muestra nueva. Mayor = más rápido. |
| `EMA_SNAP_MM` | `600` | Salto directo si la mediana se aleja más de esto. |

### Decisión

| Macro | Valor por defecto | Descripción |
|---|---|---|
| `UMBRAL_OBSTACULO_MM` | `1000` | Bajar de aquí estando libre → obstáculo. |
| `UMBRAL_LIBRE_MM` | `1300` | Superar esto estando obstruido → libre. |
| `FRAMES_PARA_OBSTACULO` | `2` | Frames consecutivos para declarar obstáculo. |
| `FRAMES_PARA_LIBRE` | `4` | Frames consecutivos para declarar libre. |

---

## Salidas

Se elige con `#define SALIDA`:

```cpp
#define SALIDA_PLOTTER 0
#define SALIDA_CSV     1
#define SALIDA         SALIDA_PLOTTER


