//**************************************************
// Proyecto 1
// Electronica Digital 1
// Andrée Herratre Paredes 21368
//**************************************************

#include <Arduino.h>
#include "driver/ledc.h"
//#include "config.h"

// configuraci[on Adafruit
//AdafruitIO_Feed *tempcanal = io.feed("Proyecto1");

#define pinADC 34     // Pin utilizado para la lectura del ADC
#define BUTTON_PIN 15 // Pin utilizado para el botón

struct Button
{
  const uint8_t PIN;
  volatile uint32_t numberKeyPresses;
  volatile bool pressed;
};

// interrupcion del boton
Button btn1 = {BUTTON_PIN, 0, false};
unsigned long button_time = 0;
unsigned long last_button_time = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

float adcRaw = 0.0;             // Almacena la lectura cruda del ADC
float promedioLecturas = 0;     // Almacena el valor promedio de las lecturas
int numLecturas = 10;           // Número de muestras para el promedio
float bufferLecturas[10];       // Buffer para las lecturas del promedio móvil
int indexLecturas = 0;          // Índice actual del buffer de lecturas
long mAvgSuma = 0;              // Suma de lecturas para el promedio móvil
long adcFiltrado = 0;           // Valor del ADC después del promedio móvil
double adcFiltradoEMA = adcRaw; // Valor del ADC después del filtro EMA
double alpha = 0.8;             // Factor de suavizado (0-1)
float temp = 0.0;
int digito;
int decenas = 0;
int unidades = 0;
int decimal = 0;
bool punto = 0;

// Parámetros de PWM para LEDs RGB
#define pwmChannel 0
#define pwmChannelR 4
#define pwmChannelG 5
#define pwmChannelB 6
#define freqPWM_RGB 5000
#define resolution_PWM 8
#define pinLedR 19
#define pinLedG 18
#define pinLedB 5
#define pinServo 17
#define freqServo 50
#define resolucionServo 16
#define servoMin 1500 // valor minimo de ciclo de trabajo
#define servoMax 8000 // valor maximo de cilo de trabajo

#define COLD_TEMP 0
#define HOT_TEMP 38

// Definir los pines del display
#define pinA 32
#define pinB 33
#define pinC 25
#define pinD 26
#define pinE 27
#define pinF 13
#define pinG 12
#define pindP

#define Display1 23
#define Display2 22
#define Display3 21

// Declaracion de las funciones globales
void promedio(void);
void mediaMovil(void);
void filtroEMA(void);
float convertToTemperature(float adcValue);
void configurarPWM(void);
void actualizarLEDsRGB(float temperatura);
void handlerBTN1ISR();
int mapServoPosition(float temperatura);
void configurarServoPWM(void);
void configurarDisplay7(void);
void desplegar7seg(int digito);
void desplayPunto(boolean punto);

// interrupcion
void handlerBTN1ISR()
{
  portENTER_CRITICAL_ISR(&mux);
  button_time = xTaskGetTickCount();
  if (button_time - last_button_time > 250)
  {
    btn1.numberKeyPresses++;
    btn1.pressed = true;
    last_button_time = button_time;
  }
  portEXIT_CRITICAL_ISR(&mux);
}


void setup()
{
  Serial.begin(115200);
  while (!Serial);
  pinMode(pinADC, INPUT);
  pinMode(btn1.PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(btn1.PIN), handlerBTN1ISR, FALLING);
  configurarPWM(); // Configurar PWM para LEDs RGB
  configurarServoPWM();
  configurarDisplay7();

  pinMode(Display1, OUTPUT);
  pinMode(Display2, OUTPUT);
  pinMode(Display3, OUTPUT);

  digitalWrite(Display1, LOW);
  digitalWrite(Display2, LOW);
  digitalWrite(Display3, LOW);

 // while(! Serial);

  //Serial.print("Connecting to Adafruit IO");

  // connect to io.adafruit.com
  //io.connect();

  // wait for a connection
 // while (io.status() < AIO_CONNECTED)
  //{
    //Serial.print(".");
    //delay(500);
  //}

  // we are connected
  //Serial.println();
  //Serial.println(io.statusText());
}

void loop()
{
  adcRaw = analogRead(pinADC); // Leer el valor crudo del ADC
  promedio();                  // Realizar el cálculo del promedio
  mediaMovil();                // Realizar el cálculo del promedio móvil

  portENTER_CRITICAL_ISR(&mux);
  if (btn1.pressed)
  {                                               // Si el botón ha sido presionado
    float voltage = convertToTemperature(adcRaw); // Convertir ADC a Celsius
    Serial.print(" Temperature: ");
    Serial.print(voltage);
    Serial.println(" °C");
    btn1.pressed = false;

    //configuracion para los valores asignados a cada display
    temp = voltage * 10;
    decenas = temp / 100;
    temp = temp - (decenas * 100);
    unidades = temp / 10;
    temp = temp - (unidades * 10);
    decimal = temp;
    // Serial.print(btn1.pressed);

    //Configuracion de Adafruit
    /*io.run();
    Serial.println("sending -> ");
    Serial.print(voltage);
    tempcanal->save(voltage);
    delay(3000);*/

    // Llamar a la función para actualizar los LEDs RGB según la temperatura
    actualizarLEDsRGB(voltage);

    // Controlar servomotor basado en la temperatura
    int mappedPosition = mapServoPosition(voltage);
    ledcWrite(pwmChannel, mappedPosition);

    btn1.pressed = false;
  }
  
  digito = decenas;
  digitalWrite(Display1, HIGH);
  digitalWrite(Display2, LOW);
  digitalWrite(Display3, LOW);
  desplegar7seg(decenas);
  delay(5);

  digito = unidades;
  digitalWrite(Display1, LOW);
  digitalWrite(Display2, HIGH);
  digitalWrite(Display3, LOW);
  desplegar7seg(unidades);
  delay(5);

  digito = decimal;
  digitalWrite(Display1, LOW);
  digitalWrite(Display2, LOW);
  digitalWrite(Display3, HIGH);
  desplegar7seg(decimal);
  delay(5);

  portEXIT_CRITICAL_ISR(&mux);

  delay(100);
}


// Funcion de ADC
void promedio(void)
{
  int sumaLecturas = 0;
  for (int k = 0; k < numLecturas; k++)
  {
    sumaLecturas += analogRead(pinADC);
    delay(1);
  }
  promedioLecturas = sumaLecturas / numLecturas;
}

// Funcion de ADC
void mediaMovil(void)
{
  adcRaw = analogReadMilliVolts(pinADC);
}

// convertir voltage a Celius
float convertToTemperature(float adcValue)
{

  float voltage = adcValue / 10;
  return voltage;
}

//PWm para los LEDs
void configurarPWM(void)
{
  ledcSetup(pwmChannelR, freqPWM_RGB, resolution_PWM);
  ledcSetup(pwmChannelG, freqPWM_RGB, resolution_PWM);
  ledcSetup(pwmChannelB, freqPWM_RGB, resolution_PWM);

  ledcAttachPin(pinLedR, pwmChannelR);
  ledcAttachPin(pinLedG, pwmChannelG);
  ledcAttachPin(pinLedB, pwmChannelB);
}

//conifiguracion del encendido y apagado de los leds dependiendo del valor de temperatura 
void actualizarLEDsRGB(float temperatura)
{
  int r = 0, g = 0, b = 0;

  if (temperatura <= 17)
  {
    // Color frío (azul)
    b = 255;
  }
  else if (temperatura > 17 && temperatura <= 18)
  {
    // Verde encendido
    g = 255;
  }
  else if (temperatura > 18)
  {
    // Rojo encendido
    r = 255;
  }

  // Aplicar los valores de PWM a los pines correspondientes
  ledcWrite(pwmChannelR, r);
  ledcWrite(pwmChannelG, g);
  ledcWrite(pwmChannelB, b);
}

//confuguración del PWM para el servomotor 
void configurarServoPWM(void)
{
  ledcSetup(pwmChannel, freqServo, resolucionServo);
  ledcAttachPin(pinServo, pwmChannel);
}

//posicionamiento del servomotor dependiendo del rango de temperatura 
int mapServoPosition(float temperatura)
{
  if (temperatura < 17)
  {
    return map(135, 0, 180, servoMin, servoMax);
  }
  else if (temperatura > 17 && temperatura <= 18)
  {
    return map(90, 0, 180, servoMin, servoMax);
  }
  else
  {
    return map(45, 0, 180, servoMin, servoMax);
  }
}


//configuracion del display en su inicializacion 
void configurarDisplay7(void)
{

  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
  pinMode(pinC, OUTPUT);
  pinMode(pinD, OUTPUT);
  pinMode(pinE, OUTPUT);
  pinMode(pinF, OUTPUT);
  pinMode(pinG, OUTPUT);

  digitalWrite(pinA, LOW);
  digitalWrite(pinB, LOW);
  digitalWrite(pinC, LOW);
  digitalWrite(pinD, LOW);
  digitalWrite(pinE, LOW);
  digitalWrite(pinF, LOW);
  digitalWrite(pinG, LOW);
}

//logica de los numeros para el display de 7 segmentos 
void desplegar7seg(int digito)
{

  switch (digito)
  {
  case 0:
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, HIGH);
    digitalWrite(pinC, HIGH);
    digitalWrite(pinD, HIGH);
    digitalWrite(pinE, HIGH);
    digitalWrite(pinF, HIGH);
    digitalWrite(pinG, LOW);
    break;
  case 1:
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
    digitalWrite(pinC, HIGH);
    digitalWrite(pinD, LOW);
    digitalWrite(pinE, LOW);
    digitalWrite(pinF, LOW);
    digitalWrite(pinG, LOW);
    break;
  case 2:
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, HIGH);
    digitalWrite(pinC, LOW);
    digitalWrite(pinD, HIGH);
    digitalWrite(pinE, HIGH);
    digitalWrite(pinF, LOW);
    digitalWrite(pinG, HIGH);
    break;
  case 3:
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, HIGH);
    digitalWrite(pinC, LOW);
    digitalWrite(pinD, HIGH);
    digitalWrite(pinE, HIGH);
    digitalWrite(pinF, LOW);
    digitalWrite(pinG, HIGH);
    break;
  case 4:
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
    digitalWrite(pinC, HIGH);
    digitalWrite(pinD, LOW);
    digitalWrite(pinE, LOW);
    digitalWrite(pinF, HIGH);
    digitalWrite(pinG, HIGH);
    break;
  case 5:
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
    digitalWrite(pinC, HIGH);
    digitalWrite(pinD, HIGH);
    digitalWrite(pinE, LOW);
    digitalWrite(pinF, HIGH);
    digitalWrite(pinG, HIGH);
    break;
  case 6:
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, HIGH);
    digitalWrite(pinC, LOW);
    digitalWrite(pinD, HIGH);
    digitalWrite(pinE, HIGH);
    digitalWrite(pinF, LOW);
    digitalWrite(pinG, HIGH);
    break;
  case 7:
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, HIGH);
    digitalWrite(pinC, HIGH);
    digitalWrite(pinD, LOW);
    digitalWrite(pinE, LOW);
    digitalWrite(pinF, LOW);
    digitalWrite(pinG, LOW);
    break;
  case 8:
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, HIGH);
    digitalWrite(pinC, HIGH);
    digitalWrite(pinD, HIGH);
    digitalWrite(pinE, HIGH);
    digitalWrite(pinF, HIGH);
    digitalWrite(pinG, HIGH);
    break;
  case 9:
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, HIGH);
    digitalWrite(pinC, HIGH);
    digitalWrite(pinD, HIGH);
    digitalWrite(pinE, LOW);
    digitalWrite(pinF, HIGH);
    digitalWrite(pinG, HIGH);
  default:
    break;
  }
}


