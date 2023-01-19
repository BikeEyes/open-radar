#include <Arduino.h>
#include "esp32-hal-gpio.h"
#include <SPI.h>

// PIN DEFINITIONS
#define PIN_I1 15
#define PIN_Q1 2

#define VCOT_PIN 25
#define EN_PIN 35
#define SS_PIN 5

#define V_REF 3.3
#define V_OUT_MAX 3
#define VCO_DIGITAL_MAX 255 * V_OUT_MAX / V_REF
#define VCO_DIGITAL_INC 5

// SPI Functions codes
#define R_DEFAULT 0x3007
#define R_ENABLE_PA 0x60
#define R_DISABLE_PA 0x1060

#define AVERAGE 2

unsigned int doppler_div = 44; // 44.44
unsigned int samples[AVERAGE];
unsigned int x;

int digital = 0;
double lfrq;
long int pp;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// void IRAM_ATTR pulseInISR()
// {

//   if (GPIO.in & (1 << 15))
//   {
//     portENTER_CRITICAL_ISR(&mux);
//     pulseDuration = esp_timer_get_time();
//     portEXIT_CRITICAL_ISR(&mux);
//     }
//   else
//   {
//     if (pulseDuration > 0)
//     {
//       portENTER_CRITICAL_ISR(&mux);
//       pulseDuration = esp_timer_get_time() - pulseDuration;
//       freqency = 1000000 / pulseDuration;
//       portEXIT_CRITICAL_ISR(&mux);
//     }
//   }
// }
volatile unsigned long interruptCounter = 0;
volatile unsigned long lastInterruptTime = 0;

void ISR()
{
  interruptCounter++;
}

void setup()
{

  Serial.begin(9600);
  Serial.println("Initializing radar");

  pinMode(SS_PIN, OUTPUT);
  digitalWrite(SS_PIN, LOW);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  SPI.beginTransaction(SPISettings(1250000, MSBFIRST, SPI_MODE0));
  SPI.transfer(R_DEFAULT);
  SPI.endTransaction();

  digitalWrite(SS_PIN, HIGH);

  Serial.println("Radar initialized");

  pinMode(PIN_I1, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_I1), ISR, RISING);
  sei();
}

void loop()
{

  if (interruptCounter > 0)
  {
    portENTER_CRITICAL_ISR(&mux);
    unsigned long currentTime = micros();
    float frequency = (float)interruptCounter / (float)(currentTime - lastInterruptTime) * 1000000.0;
    portEXIT_CRITICAL_ISR(&mux);
    float speed = frequency / doppler_div;
    if (speed > 5 && speed < 300)
    {
      Serial.print("\r\n");
      Serial.print(frequency);
      Serial.print("Hz : ");
      Serial.print(speed);
      Serial.println("km/h");
    }

    interruptCounter = 0;
    lastInterruptTime = currentTime;
  }
}