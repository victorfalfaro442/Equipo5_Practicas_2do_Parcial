/*
 Control de Velocidad
 */

#include <Arduino.h>

#define ENCODER_A GPIO_NUM_18
#define ENCODER_B GPIO_NUM_19

#define L298N_ENA GPIO_NUM_25
#define L298N_IN1 GPIO_NUM_26
#define L298N_IN2 GPIO_NUM_27

#define POT_PIN    GPIO_NUM_32
#define ACS712_PIN GPIO_NUM_34

const int PWM_FREQ = 5000;
const int PWM_CHANNEL = 0;
const int PWM_RESOLUTION = 8;
const float CPR_MOTOR = 374.0;

const bool SENTIDO_HORARIO = true;

// Límite máximo de seguridad para proteger la fuente
const int PWM_MAXIMO_SEGURO = 190; 

volatile long pulsosVelocidad = 0;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

unsigned long tiempoAnterior = 0;

void IRAM_ATTR ISR_encoder();
void setMotorUnidireccional(int pwmValue, bool sentido);

void setup() {
  Serial.begin(115200);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), ISR_encoder, RISING);

  pinMode(L298N_IN1, OUTPUT);
  pinMode(L298N_IN2, OUTPUT);

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(L298N_ENA, PWM_CHANNEL);

  pinMode(POT_PIN, INPUT);
  tiempoAnterior = millis();
}

void loop() {
  // Mapear todo el recorrido del potenciómetro (0 a 4095) al rango seguro (0 a 190)
  int valorPot = analogRead(POT_PIN);
  int pwmDeseado = map(valorPot, 0, 4095, 0, PWM_MAXIMO_SEGURO);

  if (pwmDeseado < 10) {
    pwmDeseado = 0;
  }

  setMotorUnidireccional(pwmDeseado, SENTIDO_HORARIO);

  // Cálculo de RPM cada 200 ms
  unsigned long tiempoActual = millis();
  unsigned long dt = tiempoActual - tiempoAnterior;

  if (dt >= 200) {
    long pulsosLocales = 0;
    portENTER_CRITICAL(&timerMux);
    pulsosLocales = pulsosVelocidad;
    pulsosVelocidad = 0;
    portEXIT_CRITICAL(&timerMux);

    float rpm = ((float)pulsosLocales / CPR_MOTOR) * (60000.0 / dt);
    tiempoAnterior = tiempoActual;

    Serial.print("PWM_Aplicado:"); Serial.print(pwmDeseado);
    Serial.print(" RPM_Medidas:"); Serial.println(abs(rpm));
  }
}

void IRAM_ATTR ISR_encoder() {
  portENTER_CRITICAL_ISR(&timerMux);
  if (digitalRead(ENCODER_B) == HIGH) {
    pulsosVelocidad++;
  } else {
    pulsosVelocidad--;
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setMotorUnidireccional(int pwmValue, bool sentido) {
  pwmValue = constrain(pwmValue, 0, PWM_MAXIMO_SEGURO);

  if (pwmValue > 0) {
    if (sentido) {
      digitalWrite(L298N_IN1, HIGH);
      digitalWrite(L298N_IN2, LOW);
    } else {
      digitalWrite(L298N_IN1, LOW);
      digitalWrite(L298N_IN2, HIGH);
    }
    ledcWrite(PWM_CHANNEL, pwmValue);
  } else {
    digitalWrite(L298N_IN1, LOW);
    digitalWrite(L298N_IN2, LOW);
    ledcWrite(PWM_CHANNEL, 0);
  }
}