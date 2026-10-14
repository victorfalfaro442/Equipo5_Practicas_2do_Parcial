/* Control de Posición*/

#include <Arduino.h>

#define ENCODER_A GPIO_NUM_18
#define ENCODER_B GPIO_NUM_19

#define L298N_ENA GPIO_NUM_25
#define L298N_IN1 GPIO_NUM_26
#define L298N_IN2 GPIO_NUM_27

#define POT_PIN    GPIO_NUM_32
#define ACS712_PIN GPIO_NUM_34
#define SENSOR_FC51_PIN GPIO_NUM_33 // Pin de señal digital del sensor FC-51

const int PWM_FREQ = 5000;
const int PWM_CHANNEL = 0;
const int PWM_RESOLUTION = 8;
const float CPR_MOTOR = 374.0; 

float Kp_pos = 1.5; 
const int PWM_MINIMO = 35;         
const int PWM_MAXIMO_SEGURO = 190; 

volatile long posicionPulsos = 0;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

bool sistemaCalibrado = false;

void IRAM_ATTR ISR_encoder();
int setMotorPosicion(int pwmValue);
void moverMotorConstante(int pwmValue);
void ejecutarHoming();

void setup() {
  Serial.begin(115200);
  delay(1500); // Espera de estabilización del hardware

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), ISR_encoder, RISING);

  pinMode(L298N_IN1, OUTPUT);
  pinMode(L298N_IN2, OUTPUT);

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(L298N_ENA, PWM_CHANNEL);
  
  pinMode(POT_PIN, INPUT);
  pinMode(SENSOR_FC51_PIN, INPUT_PULLUP);

  // EJECUTAR RUTINA DE HOMING AL INICIAR / REINICIAR
  ejecutarHoming();
}

void loop() {
  if (!sistemaCalibrado) return;

  // 1. Leer potenciómetro y definir objetivo en pulsos (0 a 374)
  int valorPot = analogRead(POT_PIN);
  long objetivoPulsos = map(valorPot, 0, 4095, 0, (long)CPR_MOTOR);

  // 2. Leer posición actual de forma atómica
  long posicionActual = 0;
  portENTER_CRITICAL(&timerMux);
  posicionActual = posicionPulsos;
  portEXIT_CRITICAL(&timerMux);

  // 3. Control Proporcional de posición
  long errorPulsos = objetivoPulsos - posicionActual;
  int pwmCalculado = 0;
  if (abs(errorPulsos) > 1) {
    pwmCalculado = (int)(Kp_pos * errorPulsos);
  } else {
    pwmCalculado = 0;
  }

  int pwmAplicado = setMotorPosicion(pwmCalculado);

  // 4. Monitoreo en Serial Plotter
  Serial.print("Objetivo:"); Serial.print(objetivoPulsos);
  Serial.print("\t");
  Serial.print("Actual:"); Serial.print(posicionActual);
  Serial.print("\t");
  Serial.print("Error:"); Serial.println(errorPulsos);

  delay(10);
}

void ejecutarHoming() {
  Serial.println(">>> Iniciando Homing con Sensor FC-51...");

  // Si el sensor ya está sobre la bandera al encender, retrocede un momento para liberarse
  unsigned long tiempoInicio = millis();
  while (digitalRead(SENSOR_FC51_PIN) == LOW) {
    moverMotorConstante(-120); // Retroceso seguro con fuerza
    delay(50);
    // Timeout de seguridad de 3 segundos para evitar bucles infinitos si falla el sensor
    if (millis() - tiempoInicio > 3000) break; 
  }

  // Búsqueda activa hacia adelante hasta que el pin pase a LOW (detecte la bandera)
  Serial.println("Girando en busca de la referencia cero...");
  tiempoInicio = millis();
  while (digitalRead(SENSOR_FC51_PIN) == HIGH) {
    moverMotorConstante(200); // Velocidad constante con fuerza suficiente
    delay(20);
    
    // Timeout de seguridad de 8 segundos (si el motor no encuentra la bandera, se detiene por protección)
    if (millis() - tiempoInicio > 8000) {
      Serial.println("¡Alerta! Timeout de Homing alcanzado sin detectar el sensor.");
      break;
    }
  }

  // Detener el motor al encontrar la bandera
  moverMotorConstante(0);
  
  // Establecer el cero absoluto de los pulsos
  portENTER_CRITICAL(&timerMux);
  posicionPulsos = 0;
  portEXIT_CRITICAL(&timerMux);
  
  Serial.println("¡Cero (0) establecido con éxito!");
  sistemaCalibrado = true;
  delay(500);
}

void moverMotorConstante(int pwmValue) {
  pwmValue = constrain(pwmValue, -PWM_MAXIMO_SEGURO, PWM_MAXIMO_SEGURO);

  if (pwmValue > 0) {
    digitalWrite(L298N_IN1, HIGH);
    digitalWrite(L298N_IN2, LOW);
    ledcWrite(PWM_CHANNEL, pwmValue);
  } else if (pwmValue < 0) {
    digitalWrite(L298N_IN1, LOW);
    digitalWrite(L298N_IN2, HIGH);
    ledcWrite(PWM_CHANNEL, abs(pwmValue));
  } else {
    digitalWrite(L298N_IN1, LOW);
    digitalWrite(L298N_IN2, LOW);
    ledcWrite(PWM_CHANNEL, 0);
  }
}

void IRAM_ATTR ISR_encoder() {
  portENTER_CRITICAL_ISR(&timerMux);
  if (digitalRead(ENCODER_B) == HIGH) {
    posicionPulsos++;
  } else {
    posicionPulsos--;
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

int setMotorPosicion(int pwmValue) {
  int pwmFinal = 0;
  if (pwmValue > 0) pwmFinal = pwmValue + PWM_MINIMO;
  else if (pwmValue < 0) pwmFinal = pwmValue - PWM_MINIMO;
  else pwmFinal = 0;

  pwmFinal = constrain(pwmFinal, -PWM_MAXIMO_SEGURO, PWM_MAXIMO_SEGURO);
  if (pwmValue == 0) pwmFinal = 0;

  if (pwmFinal > 0) {
    digitalWrite(L298N_IN1, HIGH);
    digitalWrite(L298N_IN2, LOW);
    ledcWrite(PWM_CHANNEL, pwmFinal);
  } else if (pwmValue < 0) {
    digitalWrite(L298N_IN1, LOW);
    digitalWrite(L298N_IN2, HIGH);
    ledcWrite(PWM_CHANNEL, abs(pwmFinal));
  } else {
    digitalWrite(L298N_IN1, LOW);
    digitalWrite(L298N_IN2, LOW);
    ledcWrite(PWM_CHANNEL, 0);
  }
  return pwmFinal;
}