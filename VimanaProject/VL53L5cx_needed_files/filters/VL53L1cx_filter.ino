/*
  VL53L1X + ESP8266/ESP32
  Detección de "camino libre" con filtro de mediana + EMA sobre un solo cono del VL53L1X (ROI 16x16, ~27°).

  Librería: "VL53L1X" by Pololu (Library Manager).
  Conexión: SDA/SCL con pull-ups, 3.3 V, GND.
  ESP8266: SDA = D2 (GPIO4), SCL = D1 (GPIO5)   |   ESP32: SDA = 21, SCL = 22

  Sin obstáculo el sensor casi nunca da una medición válida; la distancia que devuelve
  en ese caso es basura aleatoria (41..5057 mm). Por eso:
    1) Si la lectura NO es válida se guarda MAX_RANGE_MM ("no veo nada") en vez de la distancia.
    2) La mediana de las últimas N muestras actúa como votación: si la mayoría fueron
       inválidas -> MAX_RANGE_MM = camino libre.
    3) La EMA suaviza el resultado (con "snap" ante cambios grandes y reales).
    4) Histéresis + frames consecutivos para evitar oscilaciones.
*/

#include <Wire.h>
#include <VL53L1X.h>

//config --
#if defined(ESP8266)
  #define PIN_SDA 4
  #define PIN_SCL 5
#else
  #define PIN_SDA 21
  #define PIN_SCL 22
#endif
#define I2C_HZ   400000
#define BAUDIOS  115200

// Sensor
#define MODO_LARGO         true     // true = Long (hasta ~4 m), false = Short (mejor con luz ambiente)
#define TIMING_BUDGET_US   100000   // más tiempo = menos ruido (Long: mín. ~33 ms; 100 ms recomendado)
#define PERIODO_MS         100      // periodo entre mediciones (>= timing budget)
#define ROI_LADO           16       // 16 = cono completo (~27°). Mínimo 4 = cono más estrecho

// Validación
#define MAX_RANGE_MM       4000     // "sin objetivo" / camino libre
#define MIN_RANGE_MM       20
#define ACEPTAR_ESTADOS_DEBILES false  // true = acepta también estados 3, 6 y 11 (menos confiables)
#define MIN_SENAL_MCPS     0.0f     // sube (p.ej. 0.5-2.0) si aún entra ruido

// Filtro
#define VENTANA_MEDIANA    5        // impar: 3, 5, 7...  (a 10 Hz, 5 muestras = 0.5 s)
#define EMA_ALPHA          0.40f    // 0..1  (mayor = más rápido, menor = más suave)
#define EMA_SNAP_MM        600      // si |mediana - EMA| > esto, la EMA salta directo

// Decisión de camino libre (histéresis)
#define UMBRAL_OBSTACULO_MM   1500
#define UMBRAL_LIBRE_MM       1700
#define FRAMES_PARA_OBSTACULO 2
#define FRAMES_PARA_LIBRE     4

// Salida serial
#define SALIDA_PLOTTER   0   // cruda, filtrada y estado -> Serial Plotter
#define SALIDA_CSV       1   // CSV completo (mismas columnas que tu archivo + filtrada y camino_libre)
#define SALIDA           SALIDA_PLOTTER //Sólo una pequeña comodidad para obtener datos de diferentes formatos
// =============================================================

VL53L1X sensor;

// ---------- Filtro de mediana + EMA ----------
//friendly reminder! El filtro de mediana es un filtro no lineal, por lo que no se puede implementar como un simple promedio ponderado. La mediana requiere ordenar 
//los valores y seleccionar el valor central, mientras que la EMA es un promedio ponderado exponencial que da más peso a los valores recientes. La combinación de ambos 
//permite suavizar las mediciones del sensor y reducir el ruido, especialmente en entornos donde las lecturas pueden ser inconsistentes.

//Para el filtro de mediana usamos un buffer circular de tamaño de nuestra ventana, y calculamos la mediana de sus valores cada vez que se actualiza.
//Para la EMA sólo basta con su fórmula que es EMA = α * valor_actual + (1 - α) * EMA_anterior, donde α es el factor de suavizado (EMA_ALPHA).
//La histéresis se implementa con dos umbrales: uno para detectar un obstáculo y otro para detectar que el camino está libre, junto con un contador de frames consecutivos para evitar oscilaciones rápidas.

class FiltroToF {
public:
  void reset() {
    for (uint8_t i = 0; i < VENTANA_MEDIANA; i++) buf[i] = MAX_RANGE_MM; //para cada dato en el buffer, inicializamos con el valor máximo de rango, que representa "sin objetivo"
    idx = 0; ema = MAX_RANGE_MM; mediana = MAX_RANGE_MM; libre = true; //inicializamos la EMA y la mediana con el valor máximo de rango, y asumimos que el camino está libre al inicio
  }

  float actualizar(uint16_t dist, bool valida) { //dist = distancia medida por el sensor, valida = si la lectura es válida o no
    int16_t v = valida ? constrain((int)dist, MIN_RANGE_MM, MAX_RANGE_MM) : MAX_RANGE_MM; //
    buf[idx] = v; //buffer circular: guardamos el valor en la posición actual del índice, el idx es index y la v es el valor de la distancia medida o MAX_RANGE_MM si no es válida
    idx = (idx + 1) % VENTANA_MEDIANA; //incrementamos el índice circularmente, de modo que cuando llegue al final del buffer vuelva a 0

    mediana = medianaVentana();

    if (fabsf((float)mediana - ema) > EMA_SNAP_MM) ema = mediana;   // cambio real: sin lag, es decir que si la diferencia entre la mediana y la EMA es mayor que el umbral de snap, la EMA se ajusta directamente a la mediana
    else ema = EMA_ALPHA * mediana + (1.0f - EMA_ALPHA) * ema; // si no, aplicamos la fórmula de la EMA para suavizar el valor de la mediana y actualizar la EMA

    if (libre  && ema < UMBRAL_OBSTACULO_MM) libre = false;         // histéresis, es decir asumimos que el camino está libre hasta que la EMA cae por debajo del umbral de obstáculo, en cuyo caso cambiamos el estado a "no libre"
    else if (!libre && ema > UMBRAL_LIBRE_MM) libre = true; // si el camino no está libre y la EMA sube por encima del umbral de libre, cambiamos el estado a "libre"

    return ema;
  }

  float   ema;
  int16_t mediana; //la mediana es un in16_t porque representa la distancia en milímetros, que es un valor entero
  bool    libre;

private:
  int16_t buf[VENTANA_MEDIANA];
  uint8_t idx;

  int16_t medianaVentana() {
    int16_t t[VENTANA_MEDIANA]; 
    memcpy(t, buf, sizeof(t));
    for (uint8_t i = 1; i < VENTANA_MEDIANA; i++) {
      int16_t k = t[i]; int8_t j = i - 1;
      while (j >= 0 && t[j] > k) { t[j + 1] = t[j]; j--; } //ordenamiento por inserción, es decir que vamos comparando cada elemento con los anteriores y los vamos desplazando hasta encontrar su posición correcta dentro del buffer temporal t
      t[j + 1] = k;
    }
    return t[VENTANA_MEDIANA / 2];
  }
};

FiltroToF filtro;
bool caminoLibre = true;
uint8_t contaObs = 0, contaLibre = 0;
uint32_t muestra = 0;

// Estados Pololu: 0 = RangeValid (equivale al raw 9 de tu CSV).
// Débiles: 3 = valid min-range clipped, 6 = valid no-wrap-check fail, 11 = valid merged pulse.
bool lecturaValida(uint8_t estado, uint16_t d, float senal) {
  bool estadoOk = (estado == 0) ||
                  (ACEPTAR_ESTADOS_DEBILES && (estado == 3 || estado == 6 || estado == 11));
  if (!estadoOk) return false;
  if (d < MIN_RANGE_MM || d > MAX_RANGE_MM) return false;
  if (senal < MIN_SENAL_MCPS) return false;
  return true;
}

void setup() {
  Serial.begin(BAUDIOS);
  delay(500);
  if (SALIDA != SALIDA_PLOTTER) Serial.println(F("\nVL53L1X - filtro mediana + EMA"));

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(I2C_HZ);

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println(F("ERROR: sensor no detectado. Revisa cableado, pull-ups y 3.3V."));
    while (true) delay(1000);
  }

  sensor.setDistanceMode(MODO_LARGO ? VL53L1X::Long : VL53L1X::Short);
  sensor.setMeasurementTimingBudget(TIMING_BUDGET_US);
  if (ROI_LADO < 16) sensor.setROISize(ROI_LADO, ROI_LADO);
  sensor.startContinuous(PERIODO_MS);

  filtro.reset();

  if (SALIDA == SALIDA_CSV)
    Serial.println(F("muestra,t_ms,distancia_mm,valida,estado,senal_MCPS,ambiente_MCPS,filtrada_mm,camino_libre"));
}

void loop() {
  if (!sensor.dataReady()) { delay(1); return; }
  sensor.read(false);                       // no bloqueante: ya sabemos que hay dato

  uint16_t d      = sensor.ranging_data.range_mm;
  uint8_t  estado = (uint8_t)sensor.ranging_data.range_status;
  float    senal  = sensor.ranging_data.peak_signal_count_rate_MCPS;
  float    ambi   = sensor.ranging_data.ambient_count_rate_MCPS;

  bool ok = lecturaValida(estado, d, senal);
  float f = filtro.actualizar(d, ok);

  // Decisión con conteo de frames (anti-parpadeo)
  if (!filtro.libre) { contaObs++;   contaLibre = 0; }
  else               { contaLibre++; contaObs   = 0; }

  if (caminoLibre && contaObs >= FRAMES_PARA_OBSTACULO)     caminoLibre = false;
  else if (!caminoLibre && contaLibre >= FRAMES_PARA_LIBRE) caminoLibre = true;

  muestra++;

#if SALIDA == SALIDA_PLOTTER
  Serial.print(F("CRUDA:"));     Serial.print(d);                 Serial.print(',');
  Serial.print(F("FILTRADA:"));  Serial.print((int)f);            Serial.print(',');
  Serial.print(F("VALIDA:"));    Serial.print(ok ? MAX_RANGE_MM : 0); Serial.print(',');
  Serial.print(F("LIBRE:"));     Serial.println(caminoLibre ? MAX_RANGE_MM : 0);
#else
  Serial.print(muestra);            Serial.print(',');
  Serial.print(millis());           Serial.print(',');
  Serial.print(d);                  Serial.print(',');
  Serial.print(ok ? 1 : 0);         Serial.print(',');
  Serial.print(estado);             Serial.print(',');
  Serial.print(senal, 4);           Serial.print(',');
  Serial.print(ambi, 4);            Serial.print(',');
  Serial.print((int)f);             Serial.print(',');
  Serial.println(caminoLibre ? 1 : 0);
#endif
}
