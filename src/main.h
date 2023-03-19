// DAC settings
#define DAC_ADDR 0x60
#define DAC_VREF 3.3
#define DAC_RES 4096

// RADAR settings
#define RADAR_PIN_NUMBER 4
#define RADAR_EN 10
#define RADAR_SPI_CS 12

// Radar constants
#define RADAR_VMAX 3
#define RADAR_VMIN 0
#define RADAR_MIN_FREQ 24    // GHz
#define RADAR_MAX_FREQ 24.25 // GHz
#define RADAR_SPI_SIGNAL_DEFAULT 0x3007
#define RADAR_SPI_SIGNAL_MAX 0x68
#define RADAR_SPI_ENABLE_PA 0x60
#define RADAR_SPI_DISABLE_PA 0x1060
#define RADAR_SPI_CLOCK 1250000
#define RADAR_DELTA_FREQ_GHZ RADAR_MAX_FREQ - RADAR_MIN_FREQ // GHz

// Conversions
#define GHZ_TO_MHZ 1000
#define MS_TO_S 0.001

// FMCW settings
#define SPEED_OF_LIGHT 299792458 // m/s
#define DOPPLER_DIV 44.4
#define PERIOD_SAMPLING 10       // 100 times per period
#define PULSE_AWAIT_TIME 1000000 // us

// FMCW constants

// DAC constants
#define DAC_RADAR_MAX_VAL (RADAR_VMAX / DAC_VREF * (DAC_RES - 1))

/*
  Initialize radar, send SPI signal to activate radar
*/
void initRadar();

/*
  Initialize DAC, set voltage to 0V
*/
void initDAC();

/*
  Meassure current frequency that radar is currently mixing
*/
void meassureFrequency();

/*
  Display current frequency and speed on serial monitor
*/
void displayData();

void fmSweep();
void calculateSpeed();
void calculateDistance();
void displayAdvancedData();
void enablePa();
void disablePa();
