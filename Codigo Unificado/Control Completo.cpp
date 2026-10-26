/*
 * CÓDIGO UNIFICADO FINAL: CASCADA PID, 9 CURVAS, FEEDFORWARD DE VELOCIDAD Y PWM MÁXIMO
 * Entorno: VS Code + PlatformIO (ESP32)
 */

#include <Arduino.h>

// --- DEFINICIÓN DE PINES ---
#define ENCODER_A       GPIO_NUM_18
#define ENCODER_B       GPIO_NUM_19

#define L298N_ENA       GPIO_NUM_25
#define L298N_IN1       GPIO_NUM_26
#define L298N_IN2       GPIO_NUM_27

#define SENSOR_FC51_PIN GPIO_NUM_33 // Sensor infrarrojo para Homing
#define ACS712_PIN      GPIO_NUM_34 // Sensor de corriente

// --- CONFIGURACIÓN PWM ---
const int PWM_FREQ = 5000;
const int PWM_CHANNEL = 0;
const int PWM_RESOLUTION = 8; // 8 bits -> 0 a 255

// --- VARIABLES DE CALIBRACIÓN Y ENCODER ---
volatile long posicionPulsos = 0;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
float CPR_CALIBRADO = 374.0; 
bool sistemaCalibrado = false;

// --- VARIABLES DE CORRIENTE (ACS712) ---
float offsetACS712 = 2048.0;
float corrienteFiltrada = 0.0;

// --- ESTADOS DEL SISTEMA ---
enum EstadoSistema {
  CALIBRANDO,
  MENU_ESPERA,
  CONTROL_POSICION,
  CONTROL_VELOCIDAD,
  CONTROL_CURVAS
};
EstadoSistema estadoActual = CALIBRANDO;

// --- VARIABLES DE TRAYECTORIA Y CONTROL CASCADA ---
float gradosObjetivo = 0.0;
float gradosIniciales = 0.0;
unsigned long tiempoInicioTrayectoria = 0;
float duracionTrayectoriaPos_s = 3.0;

float rpmObjetivo = 0.0;
unsigned long tiempoInicioVel_ms = 0;
float duracionTrayectoriaVel_s = 2.0;
float rpmInicial = 0.0;

// Variables para el Banco de Curvas
int perfilCurvaSeleccionado = 1;
unsigned long tiempoInicioCurva = 0;

// Ganancias PID - Lazo Interno (Velocidad)
float Kp_v = 1.2;
float Ki_v = 0.25;
float Kd_v = 0.05;
float errorV_acum = 0.0;
float errorV_anterior = 0.0;

// Ganancias PID - Lazo Externo (Posición)
float Kp_p = 1.3;  
float Kd_p = 0.17; 
float errorP_anterior = 0.0;

// Derivada del encoder para velocidad real
long ultimaPosicionEncoder = 0;
unsigned long ultimoTiempoDerivada = 0;
float derivadaEncoderFiltrada = 0.0;
float rpmActual = 0.0;

// Prototipos
void IRAM_ATTR ISR_encoder();
void aplicarMotor(int pwm);
void ejecutarCalibracionInicial();
void procesarComandosSerial();
void loopControlPosicion();
void loopControlVelocidad();
void loopControlCurvas();
void calibrarSensorCorriente();
float leerCorrienteAmps();
void actualizarVelocidadReal(unsigned long tiempoActual, long posicionActual);

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), ISR_encoder, RISING);

  pinMode(L298N_IN1, OUTPUT);
  pinMode(L298N_IN2, OUTPUT);

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(L298N_ENA, PWM_CHANNEL);
  
  pinMode(SENSOR_FC51_PIN, INPUT_PULLUP);
  pinMode(ACS712_PIN, INPUT);

  calibrarSensorCorriente();

  Serial.println("\n==================================================");
  Serial.println("INICIANDO RUTINA DE CALIBRACIÓN AUTOMÁTICA...");
  Serial.println("==================================================");
  
  ejecutarCalibracionInicial();
}

void loop() {
  unsigned long tiempoActual = millis();

  long posicionActualPulsos = 0;
  portENTER_CRITICAL(&timerMux);
  posicionActualPulsos = posicionPulsos;
  portEXIT_CRITICAL(&timerMux);

  actualizarVelocidadReal(tiempoActual, posicionActualPulsos);
  procesarComandosSerial();

  switch (estadoActual) {
    case CALIBRANDO:
      break;

    case MENU_ESPERA:
      {
        float corrienteAmps = leerCorrienteAmps();
        float gradosCrudos = ((float)posicionActualPulsos / CPR_CALIBRADO) * 360.0;
        float gradosActuales = fmod(gradosCrudos, 360.0);
        if (gradosActuales < 0) gradosActuales += 360.0;

        Serial.print(">Estado_Sistema:"); Serial.println(1); 
        Serial.print(">Grados_Actuales:"); Serial.println(gradosActuales);
        Serial.print(">Corriente_Amps:"); Serial.println(corrienteAmps);
        Serial.print(">Encoder_Pulsos:"); Serial.println(posicionActualPulsos);
        Serial.print(">Derivada_Encoder:"); Serial.println(derivadaEncoderFiltrada);
      }
      break;

    case CONTROL_POSICION:
      loopControlPosicion();
      break;

    case CONTROL_VELOCIDAD:
      loopControlVelocidad();
      break;

    case CONTROL_CURVAS:
      loopControlCurvas();
      break;
  }

  delay(15);
}

// ==========================================
// 1. CALIBRACIÓN AUTOMÁTICA DE ALTA PRECISIÓN
// ==========================================
void ejecutarCalibracionInicial() {
  aplicarMotor(0);
  delay(500);

  int pwmTest = 40;
  long posInicialTest = 0;
  
  portENTER_CRITICAL(&timerMux);
  posInicialTest = posicionPulsos;
  portEXIT_CRITICAL(&timerMux);

  while (pwmTest < 180) {
    aplicarMotor(pwmTest);
    delay(100);
    
    long posActualTest = 0;
    portENTER_CRITICAL(&timerMux);
    posActualTest = posicionPulsos;
    portEXIT_CRITICAL(&timerMux);

    if (abs(posActualTest - posInicialTest) > 3) {
      break;
    }
    pwmTest += 10;
  }

  while (digitalRead(SENSOR_FC51_PIN) == LOW) {
    aplicarMotor(-pwmTest);
    delay(20);
  }

  int pwmCrawl = 55;
  if (pwmCrawl < pwmTest) pwmCrawl = pwmTest;

  while (digitalRead(SENSOR_FC51_PIN) == HIGH) {
    aplicarMotor(pwmCrawl);
    delay(5);
  }

  portENTER_CRITICAL(&timerMux);
  posicionPulsos = 0;
  portEXIT_CRITICAL(&timerMux);

  long pulsosInicioVuelta = 0;
  portENTER_CRITICAL(&timerMux);
  pulsosInicioVuelta = posicionPulsos;
  portEXIT_CRITICAL(&timerMux);

  while (digitalRead(SENSOR_FC51_PIN) == LOW) {
    aplicarMotor(pwmTest);
    delay(5);
  }
  while (digitalRead(SENSOR_FC51_PIN) == HIGH) {
    aplicarMotor(pwmTest);
    delay(2);
  }

  long pulsosFinVuelta = 0;
  portENTER_CRITICAL(&timerMux);
  pulsosFinVuelta = posicionPulsos;
  portEXIT_CRITICAL(&timerMux);

  aplicarMotor(0); 

  long pulsosCalculados = abs(pulsosFinVuelta - pulsosInicioVuelta);
  if (pulsosCalculados > 50) {
    CPR_CALIBRADO = (float)pulsosCalculados;
  }

  // Retorno exacto a 0 pulsos
  unsigned long tAcomodo = millis();
  while (millis() - tAcomodo < 3500) {
    long posActual = 0;
    portENTER_CRITICAL(&timerMux);
    posActual = posicionPulsos;
    portEXIT_CRITICAL(&timerMux);

    long errorPulsos = 0 - posActual;
    if (abs(errorPulsos) <= 1) break;

    float pwmCalc = 1.8 * (float)errorPulsos;
    int pwmAcomodo = constrain((int)pwmCalc, -150, 150);
    if (pwmAcomodo > 0 && pwmAcomodo < 95) pwmAcomodo = 95;
    if (pwmAcomodo < 0 && pwmAcomodo > -95) pwmAcomodo = -95;

    aplicarMotor(pwmAcomodo);
    delay(15);
  }
  
  aplicarMotor(0);
  portENTER_CRITICAL(&timerMux);
  posicionPulsos = 0;
  portEXIT_CRITICAL(&timerMux);

  sistemaCalibrado = true;
  estadoActual = MENU_ESPERA;

  Serial.println("==================================================");
  Serial.print("[CALIBRACIÓN EXITOSA] CPR Medido: "); Serial.println(CPR_CALIBRADO);
  Serial.println("¡Cero absoluto alineado! Sistema listo en Teleplot.");
  Serial.println("==================================================");
}

// ==========================================
// 2. PARSER DE COMANDOS DESDE TELEPLOT
// ==========================================
void procesarComandosSerial() {
  if (Serial.available() > 0) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();

    if (comando.length() == 0) return;

    if (comando.equalsIgnoreCase("CAL") || comando.equalsIgnoreCase("H")) {
      aplicarMotor(0);
      estadoActual = CALIBRANDO;
      Serial.println("\n[SISTEMA] Ejecutando recalibración manual...");
      ejecutarCalibracionInicial();
      return;
    }

    if (comando.equalsIgnoreCase("R")) {
      aplicarMotor(0);
      estadoActual = MENU_ESPERA;
      Serial.println("[SISTEMA] Regresando al menú de espera.");
      return;
    }

    if (comando.startsWith("C:") || comando.startsWith("c:")) {
      int perfil = comando.substring(2).toInt();
      if (perfil >= 1 && perfil <= 9) {
        perfilCurvaSeleccionado = perfil;
        tiempoInicioCurva = millis();
        errorV_acum = 0.0;
        errorV_anterior = 0.0;
        estadoActual = CONTROL_CURVAS;
        Serial.print("[BANCO DE CURVAS] Ejecutando Perfil #"); Serial.println(perfilCurvaSeleccionado);
      }
      return;
    }

    if (comando.startsWith("P:") || comando.startsWith("p:")) {
      int idxComa = comando.indexOf(',');
      if (idxComa != -1) {
        float gObj = comando.substring(2, idxComa).toFloat();
        float tObj = comando.substring(idxComa + 1).toFloat();

        if (tObj < 0.2) tObj = 0.2; 

        gradosObjetivo = gObj;
        duracionTrayectoriaPos_s = tObj;

        long posActual = 0;
        portENTER_CRITICAL(&timerMux);
        posActual = posicionPulsos;
        portEXIT_CRITICAL(&timerMux);
        
        float gradosCrudos = ((float)posActual / CPR_CALIBRADO) * 360.0;
        gradosIniciales = fmod(gradosCrudos, 360.0);
        if (gradosIniciales < 0) gradosIniciales += 360.0;

        tiempoInicioTrayectoria = millis();
        errorP_anterior = 0.0;
        errorV_acum = 0.0;
        errorV_anterior = 0.0;

        estadoActual = CONTROL_POSICION;
        Serial.print("[MODO POSICIÓN] Objetivo: "); Serial.print(gradosObjetivo);
        Serial.print("° en "); Serial.print(duracionTrayectoriaPos_s); Serial.println("s");
      }
    }

    if (comando.startsWith("V:") || comando.startsWith("v:")) {
      int idxComa = comando.indexOf(',');
      if (idxComa != -1) {
        float rObj = comando.substring(2, idxComa).toFloat();
        float tObj = comando.substring(idxComa + 1).toFloat();

        if (tObj < 0.2) tObj = 0.2;

        rpmObjetivo = rObj;
        duracionTrayectoriaVel_s = tObj;
        rpmInicial = rpmActual;

        tiempoInicioVel_ms = millis();
        errorV_acum = 0.0;
        errorV_anterior = 0.0;

        estadoActual = CONTROL_VELOCIDAD;
        Serial.print("[MODO VELOCIDAD] Objetivo: "); Serial.print(rpmObjetivo);
        Serial.print(" RPM en "); Serial.print(duracionTrayectoriaVel_s); Serial.println("s");
      }
    }
  }
}
// ==========================================
// 3. LAZO DE CONTROL EN CASCADA (CON CAMINO MÁS CORTO GARANTIZADO)
// ==========================================
void loopControlPosicion() {
  unsigned long tiempoActual = millis();
  float tTranscurrido = (tiempoActual - tiempoInicioTrayectoria) / 1000.0;

  // 1. Calcular el desplazamiento angular por el CAMINO MÁS CORTO desde el inicio hasta el objetivo
  float deltaTotal = gradosObjetivo - gradosIniciales;
  if (deltaTotal > 180.0)  deltaTotal -= 360.0;
  if (deltaTotal < -180.0) deltaTotal += 360.0;

  float gradosDeseadosActuales = 0;
  float velocidadFeedforwardDegS = 0; 

  if (tTranscurrido >= duracionTrayectoriaPos_s) {
    gradosDeseadosActuales = gradosIniciales + deltaTotal; // Objetivo final respetando el atajo
    velocidadFeedforwardDegS = 0;
  } else {
    float factor = tTranscurrido / duracionTrayectoriaPos_s;
    gradosDeseadosActuales = gradosIniciales + (deltaTotal * factor);
    velocidadFeedforwardDegS = deltaTotal / duracionTrayectoriaPos_s; // Velocidad con signo y dirección correcta
  }

  float rpmFeedforward = velocidadFeedforwardDegS / 6.0;

  long posicionActualPulsos = 0;
  portENTER_CRITICAL(&timerMux);
  posicionActualPulsos = posicionPulsos;
  portEXIT_CRITICAL(&timerMux);
  
  // Posición real del motor normalizada (0° a 360°)
  float gradosCrudos = ((float)posicionActualPulsos / CPR_CALIBRADO) * 360.0;
  float gradosActuales = fmod(gradosCrudos, 360.0);
  if (gradosActuales < 0) gradosActuales += 360.0;

  // Normalizar el setpoint deseado para mantenerlo limpio en el rango 0°-360°
  float setpointWrapped = fmod(gradosDeseadosActuales, 360.0);
  if (setpointWrapped < 0) setpointWrapped += 360.0;

  // 2. Cálculo del error estricto usando el camino más corto en círculo (0° - 360°)
  float errorPosicion = setpointWrapped - gradosActuales;
  if (errorPosicion > 180.0)  errorPosicion -= 360.0;
  if (errorPosicion < -180.0) errorPosicion += 360.0;

  float derivadaPosicion = errorPosicion - errorP_anterior;
  errorP_anterior = errorPosicion;

  // Lazo externo de posición
  float velocidadPedida = rpmFeedforward + (Kp_p * errorPosicion) + (Kd_p * derivadaPosicion);
  velocidadPedida = constrain(velocidadPedida, -240.0, 240.0); 

  // Lazo interno de velocidad
  float errorVelocidad = velocidadPedida - rpmActual;
  errorV_acum += errorVelocidad;
  errorV_acum = constrain(errorV_acum, -300, 300); 
  float derivadaVelocidad = errorVelocidad - errorV_anterior;
  errorV_anterior = errorVelocidad;

  // Feedforward de velocidad aplicado al PWM
  float pwmFeedforward = (velocidadPedida / 220.0) * 255.0; 
  float salidaPWM = pwmFeedforward + (Kp_v * errorVelocidad) + (Ki_v * errorV_acum) + (Kd_v * derivadaVelocidad);

  int pwmFinal = 0;
  float toleranciaLlegada = (duracionTrayectoriaPos_s > 2.5) ? 0.8 : 1.0;

  if (abs(errorPosicion) > toleranciaLlegada || abs(velocidadPedida) > 1.5) {
    int signo = (salidaPWM > 0) ? 1 : -1;
    pwmFinal = (int)abs(salidaPWM);
    if (pwmFinal > 0 && pwmFinal < 80) pwmFinal = 80; 
    pwmFinal = constrain(pwmFinal, 0, 255); 
    pwmFinal *= signo;
  } else {
    pwmFinal = 0; 
  }

  aplicarMotor(pwmFinal);
  float corrienteAmps = leerCorrienteAmps();

  // --- TELEMETRÍA EN TELEPLOT ---
  Serial.print(">Modo_Activo:"); Serial.println(2); 
  Serial.print(">Grados_Objetivo:"); Serial.println(setpointWrapped); 
  Serial.print(">Grados_Actuales:"); Serial.println(gradosActuales);
  Serial.print(">Velocidad_Pedida_RPM:"); Serial.println(velocidadPedida);
  Serial.print(">RPM_Actual:"); Serial.println(rpmActual);
  Serial.print(">PWM_Aplicado:"); Serial.println(pwmFinal);
  Serial.print(">Encoder_Pulsos:"); Serial.println(posicionActualPulsos);
  Serial.print(">Derivada_Encoder:"); Serial.println(derivadaEncoderFiltrada);
  Serial.print(">Corriente_Amps:"); Serial.println(corrienteAmps);

  delay(15);
}

// ==========================================
// 4. LAZO DE CONTROL DE VELOCIDAD CON FEEDFORWARD DIRECTO
// ==========================================
void loopControlVelocidad() {
  unsigned long tiempoActual = millis();
  float tTranscurrido = (tiempoActual - tiempoInicioVel_ms) / 1000.0;

  float rpmDeseadaActual = 0;
  if (tTranscurrido >= duracionTrayectoriaVel_s) {
    rpmDeseadaActual = rpmObjetivo;
  } else {
    rpmDeseadaActual = rpmInicial + (rpmObjetivo - rpmInicial) * (tTranscurrido / duracionTrayectoriaVel_s);
  }

  float errorVelocidad = rpmDeseadaActual - rpmActual;
  errorV_acum += errorVelocidad;
  errorV_acum = constrain(errorV_acum, -400, 400); 
  float derivadaVelocidad = errorVelocidad - errorV_anterior;
  errorV_anterior = errorVelocidad;

  // --- ACCIÓN DE ADELANTO (FEEDFORWARD) DE VELOCIDAD ---
  // Mapeamos los RPM deseados directamente a una base de PWM (asumiendo que ~220 RPM requiere el 100% debido a la caída del L298N)
  float pwmFeedforward = (rpmDeseadaActual / 220.0) * 255.0;

  // La salida combina el modelo predictivo (Feedforward) + las correcciones del PID
  float salidaPWM = pwmFeedforward + (Kp_v * errorVelocidad) + (Ki_v * errorV_acum) + (Kd_v * derivadaVelocidad);

  int pwmFinal = 0;
  if (abs(rpmDeseadaActual) > 2.0) {
    int signo = (salidaPWM > 0) ? 1 : -1;
    pwmFinal = (int)abs(salidaPWM);
    if (pwmFinal > 0 && pwmFinal < 95) pwmFinal = 95; // Superar fricción estática
    pwmFinal = constrain(pwmFinal, 0, 255);            // Liberado al 100%
    pwmFinal *= signo;
  } else {
    pwmFinal = 0;
  }

  aplicarMotor(pwmFinal);
  float corrienteAmps = leerCorrienteAmps();

  long posicionActualPulsos = 0;
  portENTER_CRITICAL(&timerMux);
  posicionActualPulsos = posicionPulsos;
  portEXIT_CRITICAL(&timerMux);
  
  float gradosCrudos = ((float)posicionActualPulsos / CPR_CALIBRADO) * 360.0;
  float gradosActuales = fmod(gradosCrudos, 360.0);
  if (gradosActuales < 0) gradosActuales += 360.0;

  Serial.print(">Modo_Activo:"); Serial.println(3); 
  Serial.print(">RPM_Objetivo:"); Serial.println(rpmDeseadaActual);
  Serial.print(">RPM_Actual:"); Serial.println(rpmActual);
  Serial.print(">PWM_Aplicado:"); Serial.println(pwmFinal);
  Serial.print(">Grados_Actuales:"); Serial.println(gradosActuales);
  Serial.print(">Encoder_Pulsos:"); Serial.println(posicionActualPulsos);
  Serial.print(">Derivada_Encoder:"); Serial.println(derivadaEncoderFiltrada);
  Serial.print(">Corriente_Amps:"); Serial.println(corrienteAmps);

  delay(20);
}

// ==========================================
// 5. BANCO DE 9 CURVAS SELECCIONABLES VÍA TELEPLOT
// ==========================================
void loopControlCurvas() {
  unsigned long tiempoActual = millis();
  float t = (tiempoActual - tiempoInicioCurva) / 1000.0;
  
  float consignaPWM = 100;

  switch (perfilCurvaSeleccionado) {
    case 1: consignaPWM = 180; break;
    case 2: consignaPWM = 100.0 + fmod(t, 10.0) * 10.0; break;
    case 3: 
      {
        float ciclo = fmod(t, 12.0);
        if (ciclo < 3.0) consignaPWM = 100.0 + (ciclo / 3.0) * 100.0;
        else if (ciclo >= 3.0 && ciclo < 9.0) consignaPWM = 255.0;
        else consignaPWM = 255.0 - ((ciclo - 9.0) / 3.0) * 155.0;
      }
      break;
    case 4: consignaPWM = 150.0 + 100.0 * sin(2.0 * 3.14159 * 0.3 * t); break;
    case 5: 
      {
        int periodo = (int)(t / 3.0);
        consignaPWM = (periodo % 2 == 0) ? 255 : 110;
      }
      break;
    case 6: 
      {
        float k = fmod(t, 10.0) / 10.0 * 3.0;
        consignaPWM = 100.0 + 155.0 * (exp(k) - 1.0) / (exp(3.0) - 1.0);
      }
      break;
    case 7: 
      {
        float t0 = 5.0, sigma = 1.2;
        float cicloT = fmod(t, 10.0);
        consignaPWM = 100.0 + 155.0 * exp(-pow((cicloT - t0) / sigma, 2));
      }
      break;
    case 8: 
      {
        float T = 4.0;
        consignaPWM = 100.0 + (fmod(t, T) / T) * 155.0;
      }
      break;
    case 9: 
      {
        if ((int)(t * 5) != (int)((t - 0.02) * 5)) {
          consignaPWM = random(100, 255);
        }
      }
      break;
  }

  int pwmFinal = constrain((int)consignaPWM, 0, 255);
  if (pwmFinal > 0 && pwmFinal < 95) pwmFinal = 95; 

  aplicarMotor(pwmFinal);

  long posicionActualPulsos = 0;
  portENTER_CRITICAL(&timerMux);
  posicionActualPulsos = posicionPulsos;
  portEXIT_CRITICAL(&timerMux);
  
  float gradosCrudos = ((float)posicionActualPulsos / CPR_CALIBRADO) * 360.0;
  float gradosActuales = fmod(gradosCrudos, 360.0);
  if (gradosActuales < 0) gradosActuales += 360.0;

  Serial.print(">Modo_Activo:"); Serial.println(4); 
  Serial.print(">Perfil_Activo:"); Serial.println(perfilCurvaSeleccionado);
  Serial.print(">Tiempo_s:"); Serial.println(t);
  Serial.print(">PWM_Aplicado:"); Serial.println(pwmFinal);
  Serial.print(">Grados_Actuales:"); Serial.println(gradosActuales);
  Serial.print(">Encoder_Pulsos:"); Serial.println(posicionActualPulsos);
  Serial.print(">Derivada_Encoder:"); Serial.println(derivadaEncoderFiltrada);
  Serial.print(">Corriente_Amps:"); Serial.println(leerCorrienteAmps());

  delay(20);
}

// ==========================================
// FUNCIONES AUXILIARES
// ==========================================
void actualizarVelocidadReal(unsigned long tiempoActual, long posicionActual) {
  unsigned long deltaTiempoMs = tiempoActual - ultimoTiempoDerivada;
  if (deltaTiempoMs >= 25) {
    long deltaPulsos = posicionActual - ultimaPosicionEncoder;
    float derivadaInstantanea = ((float)deltaPulsos / (float)deltaTiempoMs) * 1000.0; 
    derivadaEncoderFiltrada = (0.7 * derivadaEncoderFiltrada) + (0.3 * derivadaInstantanea);
    rpmActual = (derivadaEncoderFiltrada * 60.0) / CPR_CALIBRADO;

    ultimaPosicionEncoder = posicionActual;
    ultimoTiempoDerivada = tiempoActual;
  }
}

void calibrarSensorCorriente() {
  long sumaLecturas = 0;
  int muestras = 200;
  for (int i = 0; i < muestras; i++) {
    sumaLecturas += analogRead(ACS712_PIN);
    delay(3);
  }
  offsetACS712 = (float)sumaLecturas / (float)muestras;
}

float leerCorrienteAmps() {
  long sumaMuestras = 0;
  int numMuestras = 10;
  for (int i = 0; i < numMuestras; i++) {
    sumaMuestras += analogRead(ACS712_PIN);
  }
  float adcPromedio = (float)sumaMuestras / (float)numMuestras;
  float voltajeMuestreado = (adcPromedio / 4095.0) * 3.3;
  float voltajeOffset = (offsetACS712 / 4095.0) * 3.3;
  float sensibilidad = 0.185; 
  float corrienteInstantanea = (voltajeMuestreado - voltajeOffset) / sensibilidad;

  corrienteFiltrada = (0.8 * corrienteFiltrada) + (0.2 * corrienteInstantanea);
  if (abs(corrienteFiltrada) < 0.03) corrienteFiltrada = 0.0;

  return corrienteFiltrada;
}

// ==========================================
// INTERRUPCIÓN Y MOTOR
// ==========================================
void IRAM_ATTR ISR_encoder() {
  portENTER_CRITICAL_ISR(&timerMux);
  if (digitalRead(ENCODER_B) == HIGH) {
    posicionPulsos++;
  } else {
    posicionPulsos--;
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

void aplicarMotor(int pwm) {
  if (pwm > 0) {
    digitalWrite(L298N_IN1, HIGH);
    digitalWrite(L298N_IN2, LOW);
    ledcWrite(PWM_CHANNEL, abs(pwm));
  } else if (pwm < 0) {
    digitalWrite(L298N_IN1, LOW);
    digitalWrite(L298N_IN2, HIGH);
    ledcWrite(PWM_CHANNEL, abs(pwm));
  } else {
    digitalWrite(L298N_IN1, LOW);
    digitalWrite(L298N_IN2, LOW);
    ledcWrite(PWM_CHANNEL, 0);
  }
}