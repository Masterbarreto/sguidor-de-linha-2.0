#include <Arduino.h>

// Definição dos pinos de controle do motor
#define M1 9     // PinoVelocidade 1º Motor ( 0 a 255) Porta IN2 ponte H;
#define M2 11    //PinoVelocidade 2º Motor ( 0 a 255)  Porta IN4 ponte H;
#define dir1 8   //PinoDireção do 1º Motor: Para frente / Para trás (HIGH ou LOW) porta IN1 ponte H;
#define dir2 10  //PinoDireção do 2º Motor: Para frente / Para trás (HIGH ou LOW) porta IN3 ponte H;

// Definição dos pinos dos sensores
#define pin_S1 7
#define pin_S2 6
#define pin_S3 5

// Definição dos parâmetros do PID
float kp = 0.5;
float ki = 0.01;
float kd = 0.05;

// Variáveis
bool Sensor1, Sensor2, Sensor3;
float error, integral, derivative, previousError;
int velocidadeLow = 50, velocidadeHigh = 255, velocidadeCurva = 0;

void setup() {
  // Setamos os pinos de controle dos motores como saída
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  // Setamos a direção inicial do motor como 0, isso fará com que ambos os motores girem para frente
  digitalWrite(dir1, LOW);
  digitalWrite(dir2, LOW);
  // Setamos os pinos dos sensores como entrada
  pinMode(pin_S1, INPUT);
  pinMode(pin_S2, INPUT);
  pinMode(pin_S3, INPUT);
}

void loop() {
  // Leitura dos valores dos sensores
  Sensor1 = digitalRead(pin_S1);
  Sensor2 = digitalRead(pin_S2);
  Sensor3 = digitalRead(pin_S3);

  // Cálculo do erro
  error = (float)sensorValues[3] - (float)sensorValues[4];

  // Cálculo da derivada
  derivative = (error - previousError) / 0.02;

  // Cálculo da integral
  integral += error * 0.02;

  // Cálculo da ação do PID
  float output = kp * error + ki * integral + kd * derivative;

  // Saturação da saída
  if (output > 255) {
    output = 255;
  } else if (output < 0) {
    output = 0;
  }

  // Controle dos motores
  motor.setSpeedLeft(speed + output);
  motor.setSpeedRight(speed - output);

  // Atualização do erro anterior
  previousError = error;

  // Lógica de comportamento do robô
  if ((Sensor1 == 0) && (Sensor2 == 1) && (Sensor3 == 0)) {  // Se detectar na extremidade das faixas duas cores brancas
    analogWrite(M1, velocidadeHigh);           // Ambos motores ligam na mesma velocidade
    analogWrite(M2, velocidadeHigh);
  }

  if ((Sensor1 == 1) && (Sensor2 == 0) && (Sensor3 == 0)) {  // Se detectar um lado preto e o outro branco
    analogWrite(M2, 0);                    // O motor 1 desliga
    analogWrite(M1, velocidadeHigh);           // O motor 2 fica ligado, fazendo assim o carrinho virar
  }

  if ((Sensor1 == 0) && (Sensor2 == 0) && (Sensor3 == 1)) {  // Se detectar um lado branco e o outro preto
    analogWrite(M2, velocidadeHigh);           //O motor 1 fica ligado
    analogWrite(M1, 0);                    // O motor 2 desliga, fazendo assim o carrinho virar no outro sentido
  }

  if ((Sensor1 == 1) && (Sensor2 == 1) && (Sensor3 == 1)) {  // Se detectar preto em todos os sensores
    analogWrite(M1, velocidadeCurva);          // Ambos os motores giram em velocidade baixa
    analogWrite(M2, velocidadeCurva);
  }
}
