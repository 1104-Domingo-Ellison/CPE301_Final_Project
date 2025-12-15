// Author: Ellison Domingo (Group 47)
// Due 12-13-25
// CPE 301 Final Project - Water Cooler

#include <DHT11.h> // Arduino library for DHT11
#include <LiquidCrystal.h> // Arduino library for LCD allowed
#include <Stepper.h> // Arduino library for stepper motor allowed

#define RDA 0x80
#define TBE 0x20

volatile unsigned char* ddr_b = (unsigned char*)0x24;
volatile unsigned char* port_b = (unsigned char*)0x25;
volatile unsigned char* ddr_h = (unsigned char*)0x101;
volatile unsigned char* port_h = (unsigned char*)0x102;

volatile unsigned char* ddr_e = (unsigned char*)0x2D;
volatile unsigned char* port_e = (unsigned char*)0x2E;
volatile unsigned char* pin_e = (unsigned char*)0x2C;
volatile unsigned char* ddr_g = (unsigned char*)0x33;
volatile unsigned char* port_g = (unsigned char*)0x34;

inline void setPH(uint8_t bit) { *port_h |= (1 << bit); }
inline void clrPH(uint8_t bit) { *port_h &= ~(1 << bit); }
inline void setPB(uint8_t bit) { *port_b |= (1 << bit); }
inline void clrPB(uint8_t bit) { *port_b &= ~(1 << bit); }

inline void setPE(uint8_t bit) { *port_e |= (1 << bit); }
inline void clrPE(uint8_t bit) { *port_e &= ~(1 << bit); }
inline void setPG(uint8_t bit) { *port_g |= (1 << bit); }
inline void clrPG(uint8_t bit) { *port_g &= ~(1 << bit); }

unsigned long lastStartDebounceTime = 0;
unsigned long lastResetDebounceTime = 0;
int startButtonState = 0;
int resetButtonState = 0;
int lastStartReading = 1;
int lastResetReading = 1;

// UART Register Addresses
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int  *)0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

// ADC Register Addresses
volatile unsigned char *my_ADMUX   = (unsigned char *)0x007C;
volatile unsigned char *my_ADCSRB  = (unsigned char *)0x007B;
volatile unsigned char *my_ADCSRA  = (unsigned char *)0x007A;
volatile unsigned int  *my_ADC_DATA = (unsigned int  *)0x0078;

// Function Prototypes
void U0init(int U0baud);
unsigned char U0kbhit();
unsigned char U0getchar();
void U0putchar(unsigned char c);
void adc_init();
unsigned int adc_read(unsigned char adc_channel_num);
static void putstr(const char *s);
static void print_uint_dec(unsigned int v);

// Threshold setup for water sensor
static const unsigned int THRESHOLD = 170; // adjust per sensor calibration

unsigned long startMillis;
unsigned long currentMillis;

const int dhtPin = 22;
DHT11 dht11(dhtPin);
int temperature = 0;
int humidity = 0;


const int RS = 27, EN = 26, D4 = 34, D5 = 35, D6 = 36, D7 = 37;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

const int potPin = A0;
const int stepsPerRevolution = 2048;
const int rpm = 10;
Stepper stepperMotor = Stepper(stepsPerRevolution, 38, 40, 39, 41);
int currentPot = 0;
int prevPot = 0;

byte degreeChar[8] = {
  0b11100,
  0b10100,
  0b11100,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};

byte percentChar[8] = {
  0b11001,
  0b11010,
  0b00010,
  0b00100,
  0b00100,
  0b01000,
  0b01011,
  0b10011
};


bool disabled = false;
bool idle = false;
bool running = false;
bool error = false;

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(9600);
  U0init(9600);
  lcd.begin(16, 2); // set up number of columns and rows

  lcd.createChar(1, degreeChar);
  lcd.createChar(2, percentChar);

  stepperMotor.setSpeed(rpm);

  //pinMode(8, OUTPUT);
  //pinMode(9, OUTPUT);
  *ddr_h |= (1 << 5) | (1 << 6);
  //pinMode(10, OUTPUT);
  //pinMode(11, OUTPUT);
  *ddr_b |= (1 << 4) | (1 << 5);

  //pinMode(2, INPUT);
  //pinMode(3, INPUT);
  *ddr_e &= ~((1 << 4) | (1 << 5));

  adc_init();

  startMillis = millis();

  disabled = true;

  //pinMode(5, OUTPUT);
  *ddr_e |= (1 << 3);
  //pinMode(6, OUTPUT);
  //pinMode(7, OUTPUT);
  *ddr_h |= (1 << 3) | (1 << 4);

  lcd.setCursor(0, 0);
  lcd.print("Setup complete!");
  Serial.print("setup complete");
}

void loop()
{
  // ALL STATES EXCEPT DISABLED
  if (!disabled)
  {
    // Humidity and Temperature monitored and reported
    currentMillis = millis();
    if (currentMillis - startMillis >= 60000)
    {
      // Attempt to read the temperature and humidity values from the DHT11 sensor.
      int result = dht11.readTemperatureHumidity(temperature, humidity);

      // Check the results of the readings.
      // If the reading is successful, print the temperature and humidity values.
      // If there are errors, print the appropriate error messages.
      if (result == 0 && !error)
      {
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.print(" °C\tHumidity: ");
        Serial.print(humidity);
        Serial.println(" %");

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Temp: ");
        lcd.print(temperature);
        lcd.write((byte)1);
        lcd.print("C");
        lcd.setCursor(0, 1);
        lcd.print("Humidity: ");
        lcd.print(humidity);
        lcd.write((byte)2);
      }
      else
      {
        // Print error message based on the error code.
        Serial.println(DHT11::getErrorString(result));
      }
      startMillis = currentMillis;
    }
    // Start/Stop button turns fan off and goes to DISABLED state
    //if (digitalRead(2) == HIGH)
    int currentStartReading = (*pin_e & (1 << 4)) ? 1 : 0;
    if (currentStartReading != lastStartReading)
    {
      lastStartDebounceTime = millis();
    }
    if ((millis() - lastStartDebounceTime) > 50)
    {
      if (currentStartReading != startButtonState)
      {
        startButtonState = currentStartReading;
        if (startButtonState == 0)
        {
          disabled = true;
          idle = false;
          error = false;
          running = false;

          // clear LCD
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("System Disabled");
        }
      }
    }
    lastStartReading = currentStartReading;
    // Vent position works
    currentPot = analogRead(potPin);
    int prevSteps = map(prevPot, 0, 1023, 0, 2048);
    int currentSteps = map(currentPot, 0, 1023, 0, 2048);
    if (currentSteps - prevSteps > 5 || currentSteps - prevSteps < -5)
    {
      stepperMotor.step(currentSteps - prevSteps);
    }
    prevPot = currentPot;
    //Serial.println(currentSteps - prevSteps);
  }
  
  if (disabled)
  {
    // yellow LED on
    //digitalWrite(8, HIGH);
    //digitalWrite(9, LOW);
    //digitalWrite(10, LOW);
    //digitalWrite(11, LOW);
    setPH(5);
    clrPH(6);
    clrPB(4);
    clrPB(5);

    // fan turns off
    analogWrite(5, 0);
    // digitalWrite(6, LOW);
    // digitalWrite(7, LOW);
    clrPH(3);
    clrPH(4);

    // start button activates and goes to IDLE, ISR
    //if (digitalRead(2) == HIGH)
    int currentStartReading = (*pin_e & (1 << 4)) ? 1 : 0;
    if (currentStartReading != lastStartReading)
    {
      lastStartDebounceTime = millis();
    }
    if ((millis() - lastStartDebounceTime) > 50)
    {
      if (currentStartReading != startButtonState)
      {
        startButtonState = currentStartReading;
        if (startButtonState == 0)
        {
          disabled = false;
          idle = true;
        }
      }
    }
    lastStartReading = currentStartReading;
  }
  else if (idle)
  {
    // green LED on
    //digitalWrite(8, LOW);
    //digitalWrite(9, HIGH);
    //digitalWrite(10, LOW);
    //digitalWrite(11, LOW);
    clrPH(5);
    setPH(6);
    clrPB(4);
    clrPB(5);

    // fan turns off
    analogWrite(5, 0);
    // digitalWrite(6, LOW);
    // digitalWrite(7, LOW);
    clrPH(3);
    clrPH(4);

    // Check water level and change to error if too low
    unsigned int val = adc_read(1); // read from channel 1 (A1)
    // check against water threshold and print accordingly
    if (val < THRESHOLD) {
      putstr("Water Level too low, error\n");
      idle = false;
      error = true;
    } else {
      putstr("Current water level: ");
      print_uint_dec(val);
      U0putchar('\n');
    }
    // check against temp/humid level
    if (temperature >= 26)
    {
      idle = false;
      running = true;
    }
  }
  else if (error)
  {
    // fan turns off
    analogWrite(5, 0);
    // digitalWrite(6, LOW);
    // digitalWrite(7, LOW);
    clrPH(3);
    clrPH(4);

    // red LED on
    //digitalWrite(8, LOW);
    //digitalWrite(9, LOW);
    //digitalWrite(10, HIGH);
    //digitalWrite(11, LOW);
    clrPH(5);
    clrPH(6);
    setPB(4);
    clrPB(5);

    // error message on LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Error: Water");
    lcd.setCursor(0, 1);
    lcd.print("level too low!");
    

    // reset button returns to IDLE
    //if (digitalRead(3) == HIGH)
    int currentResetReading = (*pin_e & (1 << 5)) ? 1 : 0;
    if (currentResetReading != lastResetReading)
    {
      lastResetDebounceTime = millis();
    }
    if ((millis() - lastResetDebounceTime) > 50)
    {
      if (currentResetReading != resetButtonState)
      {
        resetButtonState = currentResetReading;
        if (resetButtonState == 0)
        {
          idle = true;
          error = false;
        }
      }
    }
    lastResetReading = currentResetReading;
  }
  else if (running)
  {
    // blue LED on
    //digitalWrite(8, LOW);
    //digitalWrite(9, LOW);
    //digitalWrite(10, LOW);
    //digitalWrite(11, HIGH);
    clrPH(5);
    clrPH(6);
    clrPB(4);
    setPB(5);

   
    // fan motor on
    analogWrite(5, 150);
    // digitalWrite(6, HIGH);
    // digitalWrite(7, LOW);
    setPH(3);
    clrPH(4);

    // switch back to IDLE following temp/humid levels
    if (temperature < 26)
    {
      analogWrite(5, 0);
      // digitalWrite(6, LOW);
      // digitalWrite(7, LOW);
      clrPH(3);
      clrPH(4);
      running = false;
      idle = true;
    }
  }
}


// ---------------------- ADC SETUP -------------------------
void adc_init() {
  // Enable ADC, disable auto trigger & interrupt, set prescaler /128
  *my_ADCSRA = (1 << 7) | (1 << 2) | (1 << 1) | (1 << 0);
  // Clear MUX5 and ADTS bits (free-running mode off)
  *my_ADCSRB &= ~((1 << 3) | (1 << 2) | (1 << 1) | (1 << 0));
  // Reference = AVCC, right-adjusted, channel = 0
  *my_ADMUX = (1 << 6);
}

// ---------------------- ADC READ --------------------------
unsigned int adc_read(unsigned char adc_channel_num) {
  adc_channel_num &= 0x07;              // limit to 0–7
  *my_ADMUX &= 0xE0;                    // clear channel bits
  *my_ADMUX |= adc_channel_num;         // select channel
  *my_ADCSRB &= ~(1 << 3);              // MUX5 = 0
  *my_ADCSRA |= (1 << 6);               // start conversion
  while ((*my_ADCSRA & (1 << 6)) != 0); // wait until done
  unsigned int val = *my_ADC_DATA & 0x03FF; // read 10-bit result
  return val;
}

// ---------------------- UART CONFIG -----------------------
void U0init(int U0baud) {
  unsigned long FCPU = 16000000;
  unsigned int tbaud = (FCPU / 16 / U0baud - 1);
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0  = tbaud;
}
unsigned char U0kbhit()   { return *myUCSR0A & RDA; }
unsigned char U0getchar() { return *myUDR0; }
void U0putchar(unsigned char c) {
  while((*myUCSR0A & TBE) == 0);
  *myUDR0 = c;
}

// ---------------------- HELPER PRINTS ---------------------
static void putstr(const char *s) {
  while (*s) U0putchar((unsigned char)*s++);
}
static void print_uint_dec(unsigned int v) {
  char buf[5];
  int i = 0;
  if (v == 0) { U0putchar('0'); return; }
  while (v > 0 && i < 5) {
    buf[i++] = '0' + (v % 10);
    v /= 10;
  }
  while (i--) U0putchar(buf[i]);
}