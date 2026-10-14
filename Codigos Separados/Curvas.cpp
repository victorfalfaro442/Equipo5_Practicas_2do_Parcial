/*Cambio de curvas por tiempo con graficas*/

#include <Arduino.h>

#define ENCODER_A GPIO_NUM_18
#define ENCODER_B GPIO_NUM_19

#define L298N_ENA GPIO_NUM_25
#define L298N_IN1 GPIO_NUM_26
#define L298N_IN2 GPIO_NUM_27

const int PWM_FREQ = 5000;
const int PWM_CHANNEL = 0;
const int PWM_RESOLUTION = 8;

unsigned long tiempoInicioPerfil = 0;
unsigned long tiempoUltimoCambio = 0;
int perfilActual = 1;
const unsigned long INTERVALO_PERFIL_MS = 15000; // 15 segundos por cada perfil

void aplicarMotor(int pwm);

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  pinMode(L298N_IN1, OUTPUT);
  pinMode(L298N_IN2, OUTPUT);

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(L298N_ENA, PWM_CHANNEL);

  Serial.println("\n==================================================");
  Serial.println("SECUENCIADOR DE LOS 9 PERFILES (15s c/u, PWM min 100)");
  Serial.println("==================================================");

  tiempoInicioPerfil = millis();
  tiempoUltimoCambio = millis();
}

void loop() {
  unsigned long tiempoActual = millis();

  // Cambiar de perfil automáticamente cada 15 segundos
  if (tiempoActual - tiempoUltimoCambio >= INTERVALO_PERFIL_MS) {
    tiempoUltimoCambio = tiempoActual;
    tiempoInicioPerfil = tiempoActual; 

    perfilActual++;
    if (perfilActual > 9) {
      perfilActual = 1; // Vuelve al perfil 1 al terminar el 9 (bucle continuo)
    }

    Serial.print("\n>>> CAMBIO AUTOMÁTICO A PERFIL: ");
    Serial.println(perfilActual);
  }

  // Tiempo local transcurrido en segundos dentro del perfil actual (0 a 15s)
  float t = (tiempoActual - tiempoInicioPerfil) / 1000.0;
  float consignaPWM = 100;

  // --- MATEMÁTICA DE LOS 9 PERFILES (Ajustados con base mínima de 100) ---
  switch (perfilActual) {
    case 1: // 1. Constante (Fijo en 200)
      consignaPWM = 200;
      break;

    case 2: // 2. Rampa lineal (Escala de 100 a 200 en los 15s)
      consignaPWM = 100.0 + (t / 15.0) * 100.0;
      break;

    case 3: // 3. Perfil trapezoidal (Sube a 230, se mantiene y baja a 100)
      if (t < 3.0) {
        consignaPWM = 100.0 + (t / 3.0) * 130.0;
      } else if (t >= 3.0 && t < 12.0) {
        consignaPWM = 230.0;
      } else {
        consignaPWM = 230.0 - ((t - 12.0) / 3.0) * 130.0;
      }
      break;

    case 4: // 4. Senoidal (Oscila suavemente entre 100 y 200)
      consignaPWM = 150.0 + 50.0 * sin(2.0 * 3.14159 * 0.4 * t);
      break;

    case 5: // 5. Escalonada / Onda cuadrada (Alterna entre 120 y 200 cada 3.75s)
      {
        int periodo = (int)(t / 3.75);
        if (periodo % 2 == 0) consignaPWM = 200;
        else consignaPWM = 120;
      }
      break;

    case 6: // 6. Exponencial creciente (Crece suavemente de 100 a 200)
      {
        float k = (t / 15.0) * 4.0;
        consignaPWM = 100.0 + 100.0 * (exp(k) - 1.0) / (exp(4.0) - 1.0);
      }
      break;

    case 7: // 7. Perfil Gaussiano (Pulso centrado en t = 7.5s, oscila entre 100 y 200)
      {
        float t0 = 7.5;
        float sigma = 1.5;
        consignaPWM = 100.0 + 100.0 * exp(-pow((t - t0) / sigma, 2));
      }
      break;

    case 8: // 8. Diente de sierra (Ciclos de 5s que van de 100 a 200)
      {
        float T = 5.0;
        float ciclo = fmod(t, T) / T;
        consignaPWM = 100.0 + ciclo * 100.0;
      }
      break;

    case 9: // 9. Modulación por ruido blanco (Valores aleatorios seguros entre 100 y 200)
      {
        consignaPWM = random(100, 201);
        delay(120); 
      }
      break;
  }

  // Restricción estricta de seguridad: nunca menor a 100, nunca mayor a 230
  int pwmFinal = constrain((int)consignaPWM, 100, 230);
  aplicarMotor(pwmFinal);

  // --- TRAMAS PARA TELEPLOT ---
  Serial.print(">Perfil_Activo:"); Serial.println(perfilActual);
  Serial.print(">Tiempo_en_Perfil_s:"); Serial.println(t);
  Serial.print(">PWM_Consigna:"); Serial.println(pwmFinal);

  delay(20);
}

void aplicarMotor(int pwm) {
  if (pwm > 0) {
    digitalWrite(L298N_IN1, HIGH);
    digitalWrite(L298N_IN2, LOW);
    ledcWrite(PWM_CHANNEL, pwm);
  } else {
    digitalWrite(L298N_IN1, LOW);
    digitalWrite(L298N_IN2, LOW);
    ledcWrite(PWM_CHANNEL, 0);
  }
}