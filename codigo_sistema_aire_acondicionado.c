// -------- CONFIGURACION DE SENSORES --------
// LM35: salida analogica de 10 mV por cada grado C. No mide humedad.
const int PIN_TEMP = 34;  // ADC1 (GPIO34). Tambien puedes usar el 4 si no usas WiFi

// KY-038: conectar pin DO al GPIO 19
const int PIN_SONIDO = 19;

// KY-033: sensor infrarrojo reflectivo (salida digital)
const int PIN_PRESENCIA = 18;

// -------- L298N --------
const int ENA = 23;
const int IN1 = 16;
const int IN2 = 17;

// KY-038: normalmente da HIGH al detectar sonido (false).
// Cambiar a true si en tu modulo ocurre al reves.
const bool SONIDO_ACTIVO_BAJO = false;

// KY-033: normalmente da LOW cuando detecta un objeto cercano.
// Cambiar a false si en tu modulo ocurre al reves.
const bool PRESENCIA_ACTIVO_BAJO = true;

// -------- PARAMETROS DEL LM35 --------
const float MV_POR_GRADO = 10.0;     // 10 mV por cada grado C
const float OFFSET_TEMP = 6.0;       // Correccion manual en C (calibracion)
const int MUESTRAS_ADC = 10;         // Promedio para reducir ruido

// -------- PARAMETROS --------
// Apagar tras 10 segundos sin actividad
const unsigned long TIEMPO_INACTIVIDAD = 10UL * 1000UL;

// PWM de 8 bits: valores entre 0 y 255 (mayor potencia para compensar caida del L298N)
const int PWM_MINIMO = 180;
const int PWM_MAXIMO = 255;

// -------- VARIABLES --------
bool sistemaActivo = false;
bool zonaLiberada = true;

// Bandera para la interrupcion del KY-038
volatile bool pulsoSonido = false;

unsigned long ultimaActividad = 0;
unsigned long ultimaLecturaTemp = 0;
unsigned long ultimoEventoSonido = 0;

const unsigned long INTERVALO_TEMP = 1000;
const unsigned long FILTRO_SONIDO = 500;

float temperatura = NAN;
int velocidadVentilador = 0;

// -------- INTERRUPCION DE SONIDO --------
void IRAM_ATTR detectarPulsoSonido() {
  pulsoSonido = true;
}

// -------- LEER TEMPERATURA (LM35) --------
float leerTemperatura() {
  float suma = 0;

  for (int i = 0; i < MUESTRAS_ADC; i++) {
    suma += analogReadMilliVolts(PIN_TEMP);
    delay(2);
  }

  float mv = suma / MUESTRAS_ADC;
  float t = mv / MV_POR_GRADO + OFFSET_TEMP;

  // Valor fuera de rango razonable: sensor desconectado o mal conectado
  if (t < -5.0 || t > 120.0) {
    return NAN;
  }

  return t;
}

// -------- CONTROL DEL VENTILADOR --------
int calcularVelocidad(float t) {
  int pwm;

  if (t < 24.0) {
    pwm = 180;      // ~70% de potencia
  } else if (t < 27.0) {
    pwm = 210;      // ~82% de potencia
  } else if (t < 30.0) {
    pwm = 235;      // ~92% de potencia
  } else {
    pwm = 255;      // 100% de potencia maxima
  }

  return constrain(pwm, PWM_MINIMO, PWM_MAXIMO);
}

void controlarMotor(int pwm) {
  if (pwm <= 0) {
    ledcWrite(ENA, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    velocidadVentilador = 0;
    return;
  }

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  ledcWrite(ENA, pwm);
  velocidadVentilador = pwm;
}

// -------- INICIALIZACION --------
void setup() {
  Serial.begin(115200);

  pinMode(PIN_PRESENCIA, INPUT);
  pinMode(PIN_SONIDO, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Configurar interrupcion en el pin del KY-038 para no perder pulsos cortos
  int modoInterrupcion = SONIDO_ACTIVO_BAJO ? FALLING : RISING;
  attachInterrupt(digitalPinToInterrupt(PIN_SONIDO), detectarPulsoSonido, modoInterrupcion);

  // ADC: 12 bits. Con 6 dB el rango llega a ~2 V (el LM35 da 0.25 V a 25 C)
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_TEMP, ADC_6db);

  // Arduino-ESP32 3.x
  ledcAttach(ENA, 5000, 8);
  controlarMotor(0);

  Serial.println("Sistema de climatizacion iniciado");
  Serial.println("Esperando deteccion de presencia o sonido...");
}

// -------- CICLO PRINCIPAL --------
void loop() {
  unsigned long ahora = millis();

  // 1. DETECTAR PRESENCIA (KY-033)
  int lecturaPresencia = digitalRead(PIN_PRESENCIA);
  bool presenciaDetectada = PRESENCIA_ACTIVO_BAJO ? (lecturaPresencia == LOW) : (lecturaPresencia == HIGH);

  // Detectar una entrada al campo del sensor
  if (presenciaDetectada && zonaLiberada) {
    zonaLiberada = false;
    sistemaActivo = true;
    ultimaActividad = ahora;

    Serial.println("Presencia detectada: sistema activado");
  }

  // Rearmar la deteccion cuando la zona quede libre
  if (!presenciaDetectada) {
    zonaLiberada = true;
  }

  // 2. DETECTAR ACTIVIDAD SONORA (KY-038 por interrupcion)
  int lecturaSonido = digitalRead(PIN_SONIDO);
  bool sonidoDetectado = false;

  if (pulsoSonido) {
    pulsoSonido = false;

    if (ahora - ultimoEventoSonido >= FILTRO_SONIDO) {
      sonidoDetectado = true;
      ultimoEventoSonido = ahora;
      ultimaActividad = ahora;

      Serial.println("Sonido detectado");

      if (!sistemaActivo) {
        sistemaActivo = true;
        Serial.println("Actividad sonora: sistema activado");
      }
    }
  }

  // 3. LEER TEMPERATURA
  if (ahora - ultimaLecturaTemp >= INTERVALO_TEMP) {
    ultimaLecturaTemp = ahora;

    float t = leerTemperatura();

    if (!isnan(t)) {
      temperatura = t;
    } else {
      Serial.println("Error de lectura de temperatura (revisa el LM35)");
    }
  }

  // 4. APAGADO POR INACTIVIDAD
  if (sistemaActivo && (ahora - ultimaActividad >= TIEMPO_INACTIVIDAD)) {
    sistemaActivo = false;
    Serial.println("Sin actividad: sistema apagado");
  }

  // 5. REGULAR VELOCIDAD DEL VENTILADOR
  if (sistemaActivo) {
    if (!isnan(temperatura)) {
      int pwm = calcularVelocidad(temperatura);
      controlarMotor(pwm);
    } else {
      controlarMotor(0);
    }
  } else {
    controlarMotor(0);
  }

  // 6. MOSTRAR DATOS
  static unsigned long ultimaImpresion = 0;

  if (ahora - ultimaImpresion >= 2000) {
    ultimaImpresion = ahora;

    Serial.println("-------- ESTADO --------");

    Serial.print("Presencia: ");
    Serial.print(presenciaDetectada ? "DETECTADA" : "libre");
    Serial.print(" (lectura cruda: ");
    Serial.print(lecturaPresencia);
    Serial.println(")");

    Serial.print("Temperatura: ");
    Serial.print(temperatura);
    Serial.println(" C");

    Serial.print("Sonido (lectura cruda DO): ");
    Serial.println(lecturaSonido);

    Serial.print("Sistema: ");
    Serial.println(sistemaActivo ? "ACTIVO" : "APAGADO");

    Serial.print("PWM del ventilador: ");
    Serial.println(velocidadVentilador);
  }
}
