#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include "main.h"

// Global variables that are used to meassure frequency
unsigned int Freq = 0;

// Variable to store state of triangle wave
int PeriodTime = 0; // Time of one period of triangle wave in ms

// Triangular wave frequency

unsigned long AvgFreqRising = 0;
unsigned long AvgFreqFalling = 0;

// Calculated values
double velocity = 0;
double distance = 0;

Adafruit_MCP4725 dac;

void setup()
{
  Serial.begin(115200);
  pinMode(RADAR_PIN_NUMBER, INPUT);

  initRadar();
  initDAC();

  delay(3000);
}

void loop()
{
  meassureFrequency();
  displayData();

  fmSweep();

  Serial.print("Rising: ");
  Serial.print(AvgFreqRising);
  Serial.println(" Hz");
  Serial.print("Falling: ");
  Serial.print(AvgFreqFalling);
  Serial.println(" Hz");
  Serial.print("Period: ");
  Serial.print(PeriodTime);
  Serial.println(" ms");

  calculateSpeed();
  calculateDistance();
  displayAdvancedData();
}

void initRadar()
{
  pinMode(RADAR_SPI_CS, OUTPUT);
  pinMode(RADAR_EN, OUTPUT);
  digitalWrite(RADAR_EN, HIGH);
  digitalWrite(RADAR_SPI_CS, LOW);
  SPI.beginTransaction(SPISettings(RADAR_SPI_CLOCK, MSBFIRST, SPI_MODE0));
  SPI.transfer(RADAR_SPI_SIGNAL_MAX);
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

void meassureFrequency()
{
  unsigned int Htime = 0;
  unsigned int Ltime = 0;
  float Ttime;

  noInterrupts();
  Htime = pulseIn(RADAR_PIN_NUMBER, HIGH, PULSE_AWAIT_TIME); // read high time
  Ltime = pulseIn(RADAR_PIN_NUMBER, LOW, PULSE_AWAIT_TIME);  // read low time
  interrupts();

  Ttime = Htime + Ltime; // total time = high time + low time

  if (Ttime > 0)
  {
    Freq = 1000000 / Ttime; // calculate frequency from Ttime in Micro seconds
  }
  else
  {
    Freq = 0;
  }
}

void displayData()
{
  Serial.print("Frequency: ");
  Serial.print(Freq);
  Serial.println(" Hz");
  Serial.print("Speed: ");
  Serial.print(Freq / DOPPLER_DIV);
  Serial.println(" km/h");
}

void displayAdvancedData()
{
  Serial.print("Speed: ");
  Serial.print(velocity * 3.6);
  Serial.println(" km/h");
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" m");
}

void fmSweep()
{

  int counterStep = DAC_RADAR_MAX_VAL / PERIOD_SAMPLING;

  enablePa();

  unsigned long start = millis();

  uint32_t counter;
  /*
    Sweep from 0V to 3V
  */
  for (counter = 0; counter <= DAC_RADAR_MAX_VAL; counter += counterStep)
  {
    dac.setVoltage(counter, false);
    meassureFrequency();
    AvgFreqRising += Freq;
  }

  /*
    Sweep from 3V to 0V
  */
  for (counter; counter > 0; counter -= counterStep)
  {
    dac.setVoltage(counter, false);
    meassureFrequency();
    AvgFreqFalling += Freq;
  }

  unsigned long end = millis();

  disablePa();

  PeriodTime = (end - start) / 2;

  AvgFreqRising /= PERIOD_SAMPLING;
  AvgFreqFalling /= PERIOD_SAMPLING;
}

void calculateSpeed()
{
  long totalFreq = AvgFreqRising + AvgFreqFalling;

  double baseFreq = (double)1 / (double)(GHZ_TO_MHZ * RADAR_MIN_FREQ);

  double a = totalFreq;
  double x = (double)SPEED_OF_LIGHT / 4;
  double b = x * baseFreq;

  double y = a * b;

  velocity = y / 1000000;
}

void calculateDistance()
{
  long totalFreq = AvgFreqFalling - AvgFreqRising;

  double a = (double)totalFreq;
  double T = (double)PeriodTime * MS_TO_S;
  double x = (double)SPEED_OF_LIGHT / 4 * T;
  double y = (double)1 / (double)250;

  double b = x * y;

  Serial.print("a: ");
  Serial.println(a, 10);
  Serial.print("T: ");
  Serial.println(T, 10);
  Serial.print("x: ");
  Serial.println(x, 10);
  Serial.print("y: ");
  Serial.println(y, 10);
  Serial.print("b: ");
  Serial.println(b, 10);

  distance = a * b / 1000000;
}

void enablePa()
{
  digitalWrite(RADAR_SPI_CS, LOW);
  SPI.beginTransaction(SPISettings(RADAR_SPI_CLOCK, MSBFIRST, SPI_MODE0));
  SPI.transfer(RADAR_SPI_ENABLE_PA);
  SPI.endTransaction();
  digitalWrite(RADAR_SPI_CS, HIGH);
  Serial.println("Enabled PA");
}

void disablePa()
{
  digitalWrite(RADAR_SPI_CS, LOW);
  SPI.beginTransaction(SPISettings(RADAR_SPI_CLOCK, MSBFIRST, SPI_MODE0));
  SPI.transfer(RADAR_SPI_DISABLE_PA);
  SPI.endTransaction();
  digitalWrite(RADAR_SPI_CS, HIGH);
  Serial.println("Disabled PA");
}