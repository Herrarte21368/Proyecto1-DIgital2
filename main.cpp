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

