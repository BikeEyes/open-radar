#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>

#define PIN_NUMBER 4
#define AVERAGE 2

#define DAC_ADDR 0x60
#define DAC_VREF 3.26
#define DAC_RES 4096

#define RADAR_VMAX 3
#define RADAR_MIN 0
#define RADAR_SPI_CS 12
#define RADAR_SPI_SIGNAL_DEFAULT 0x3007
#define RADAR_SPI_CLOCK 1250000

#define DAC_RADAR_MAX_VAL (RADAR_MAX / DAC_VREF * DAC_RES) - 1

unsigned int doppler_div = 44;
unsigned int samples[AVERAGE];
unsigned int x;
bool isRising = true;

Adafruit_MCP4725 dac;

void setup()
{
  Serial.begin(9600);
  pinMode(PIN_NUMBER, INPUT);

  initRadar();
  initDAC();
}

void loop()
{
  noInterrupts();
  pulseIn(PIN_NUMBER, HIGH);
  unsigned int pulse_length = 0;
  for (x = 0; x < AVERAGE; x++)
  {
    pulse_length = pulseIn(PIN_NUMBER, HIGH);
    pulse_length += pulseIn(PIN_NUMBER, LOW);
    samples[x] = pulse_length;
  }
  interrupts();

  // Check for consistency
  bool samples_ok = true;
  unsigned int nbPulsesTime = samples[0];
  for (x = 1; x < AVERAGE; x++)
  {
    nbPulsesTime += samples[x];
    if ((samples[x] > samples[0] * 2) || (samples[x] < samples[0] / 2))
    {
      samples_ok = false;
    }
  }

  if (samples_ok)
  {
    unsigned int Ttime = nbPulsesTime / AVERAGE;
    unsigned int Freq = 1000000 / Ttime;

    Serial.print(Ttime);
    Serial.print("\r\n");
    Serial.print(Freq);
    Serial.print("Hz : ");
    Serial.print(Freq / doppler_div);
    Serial.print("km/h\r\n");

    delay(200);
  }
}

void initRadar()
{
  pinMode(RADAR_SPI_CS, OUTPUT);
  digitalWrite(RADAR_SPI_CS, LOW);
  SPI.beginTransaction(SPISettings(RADAR_SPI_CLOCK, MSBFIRST, SPI_MODE0));
  SPI.transfer(RADAR_SPI_SIGNAL_DEFAULT);
  SPI.endTransaction();
  digitalWrite(RADAR_SPI_CS, HIGH);
  Serial.println("SPI transfered");
}

void initDAC()
{
  dac.begin(DAC_ADDR);
  // set voltage to 0V
  dac.setVoltage(0, false);
}