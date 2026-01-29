/*
  ****************************************************************************************************************
  RC transmitter
  **************
  RC transmitter with 1 to 7 channels from my repository https://github.com/stanekTM/TX_nRF24_Xch_LED
  
  Includes nRF24L01+ transceiver and ATmega328P processor.
  
  This RC transmitter works with RC receiver from my repository https://github.com/stanekTM/RX_nRF24_Motor_Servo
  
  Thanks to "Phil_G" http://www.singlechannel.co.uk for the calibration and reversal routine used in the code.
  ****************************************************************************************************************
*/

#include <RF24.h> // v1.4.11
#include <RF24_config.h>
#include <nRF24L01.h>
#include <SPI.h>
#include <EEPROM.h>

// Setting a unique address (5 bytes number or character)
const byte address[] = "jirka";

// RF communication channel setting (0-125, 2.4Ghz + 76 = 2.476Ghz)
#define RF_CHANNEL         76

// TX/RX alarm voltage setting
#define TX_BATTERY_VOLTAGE    4.2  // Maximum nominal battery voltage
#define TX_MONITORED_VOLTAGE  3.45 // Minimum battery voltage for alarm

#define RX_BATTERY_VOLTAGE    4.2  // Maximum nominal battery voltage
#define RX_MONITORED_VOLTAGE  3.45 // Minimum battery voltage for alarm

// Set the number of channels according to the controls (max 7 / A0 to A6)
#define RC_CHANNELS  2

// Control range value
#define MIN_CONTROL_VAL  1000
#define MID_CONTROL_VAL  1500
#define MAX_CONTROL_VAL  2000

// ATmega328P/PB pins overview
// PD0 - D0   PWM  328PB
// PD1 - D1   PWM  328PB
// PD2 - D2   PWM  328PB
// PD3 - D3   PWM
// PD4 - D4
// PD5 - D5   PWM
// PD6 - D6   PWM
// PD7 - D7
// PB0 - D8
// PB1 - D9   PWM
// PB2 - D10  PWM
// PB3 - D11  PWM  MOSI
// PB4 - D12       MISO
// PB5 - D13       SCK
// PC0 - D14 / A0
// PC1 - D15 / A1
// PC2 - D16 / A2
// PC3 - D17 / A3
// PC4 - D18 / A4   SDA
// PC5 - D19 / A5   SCL
// PB6 - D20        XTAL1
// PB7 - D21        XTAL2
// PC6 - D22        RESET
// PE0 - D23        328PB
// PE1 - D24        328PB
// PE2 - D25 / A6   328PB
// PE3 - D26 / A7   328PB
// ADC6   -    A6
// ADC7   -    A7

// Analog input pin array for pots (possible combination, max 7)
const byte pins_pots[] = {A0, A1, A2, A3, A4, A5, A6};

// LED alarm
#define PIN_LED           6

// Calibration button
#define PIN_BUTTON_CALIB  4

// Input battery
#define PIN_BATTERY       A7

// Pins for nRF24L01+
#define PIN_CE            9
#define PIN_CSN           10

// Hardware SPI
//          MOSI          11
//          MISO          12
//          SCK           13

// nRF24 class driver
RF24 radio(PIN_CE, PIN_CSN);

//*********************************************************************************************************************
// Sent data array (max 32 bytes)
//*********************************************************************************************************************
unsigned int rc_packet[RC_CHANNELS] = {1500};
byte rc_packet_size = RC_CHANNELS * 2; // For one control channel with a value of 1000 to 2000 we need 2 bytes(packets)

//*********************************************************************************************************************
// Structure of received ACK data
//*********************************************************************************************************************
struct telemetry_packet_size
{
  byte rssi;    // Not used yet
  byte batt_A1 = 255;
  byte batt_A2; // Not used yet
};
telemetry_packet_size telemetry_packet;

//*********************************************************************************************************************
// Read pots
//*********************************************************************************************************************
int i, raw_pots;
int pots_value[RC_CHANNELS] = {1500};
int min_pots_calib[RC_CHANNELS] = {0};
int mid_pots_calib[RC_CHANNELS] = {512};
int max_pots_calib[RC_CHANNELS] = {1023};
byte reverse[RC_CHANNELS] = {0};

void read_pots()
{
  for (i = 0; i < RC_CHANNELS; i++)
  {
    raw_pots = analogRead(pins_pots[i]);
    
    if (raw_pots < mid_pots_calib[i])
    {
      pots_value[i] = map(raw_pots, min_pots_calib[i], mid_pots_calib[i], MIN_CONTROL_VAL, MID_CONTROL_VAL);
    }
    else
    {
      pots_value[i] = map(raw_pots, mid_pots_calib[i], max_pots_calib[i], MID_CONTROL_VAL, MAX_CONTROL_VAL);
    }
    
    if (reverse[i] == 1) pots_value[i] = (MIN_CONTROL_VAL + MAX_CONTROL_VAL) - pots_value[i];
    
    rc_packet[i] = pots_value[i];
  }
  //Serial.println(rc_packet[0]);
}

//*********************************************************************************************************************
// Calibrate or reverse the pots
//*********************************************************************************************************************
bool calibrate = 1;

void calibrate_reverse_pots()
{
  while (digitalRead(PIN_BUTTON_CALIB) == 0)
  {
    calibrate = 0;
    
    for (i = 0; i < RC_CHANNELS; i++)
    {
      raw_pots = analogRead(pins_pots[i]);
      
      if (raw_pots < min_pots_calib[i]) min_pots_calib[i] = raw_pots;
      
      if (raw_pots > max_pots_calib[i]) max_pots_calib[i] = raw_pots;
      
      mid_pots_calib[i] = raw_pots;
    }
  } // Calibrate button released
  
  if (calibrate == 0)
  {
    for (i = 0; i < RC_CHANNELS; i++)
    {
      EEPROMWriteInt(i * 6,     max_pots_calib[i]); // EEPROM locations  0,  6, 12, 18 (decimal)
      EEPROMWriteInt(i * 6 + 2, mid_pots_calib[i]); // EEPROM locations  2,  8, 14, 20 (decimal)
      EEPROMWriteInt(i * 6 + 4, min_pots_calib[i]); // EEPROM locations  4, 10, 16, 22 (decimal)
    }
    calibrate = 1;
  }
  
  for (i = 0; i < RC_CHANNELS; i++)
  {
    max_pots_calib[i] = EEPROMReadInt(i * 6);     // EEPROM locations  0,  6, 12, 18 (decimal)
    mid_pots_calib[i] = EEPROMReadInt(i * 6 + 2); // EEPROM locations  2,  8, 14, 20 (decimal)
    min_pots_calib[i] = EEPROMReadInt(i * 6 + 4); // EEPROM locations  4, 10, 16, 22 (decimal)
    reverse[i] = EEPROM.read(i + 24) & 1;         // EEPROM locations 24, 25, 26, 27 (decimal)
  }
  
  // Check for reversing, stick over on power-up
  for (i = 0; i < RC_CHANNELS; i++)
  {
    pots_value[i] = map(analogRead(pins_pots[i]), min_pots_calib[i], max_pots_calib[i], MIN_CONTROL_VAL, MAX_CONTROL_VAL);
    
    if (pots_value[i] < MIN_CONTROL_VAL + 50 || pots_value[i] > MAX_CONTROL_VAL - 50)
    {
      reverse[i] ^= B00000001;
      EEPROM.write(24 + i, reverse[i]);
    }
  }
}

//*********************************************************************************************************************
// Program setup
//*********************************************************************************************************************
void setup()
{
  //Serial.begin(9600);
  
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BATTERY, INPUT);
  pinMode(PIN_BUTTON_CALIB, INPUT_PULLUP);
  
  calibrate_reverse_pots();
  
  // Define the radio communication
  radio.begin();
  radio.setAutoAck(1);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setRetries(2, 0);
  radio.setChannel(RF_CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN); // RF24_PA_MIN (-18dBm), RF24_PA_LOW (-12dBm), RF24_PA_HIGH (-6dbm), RF24_PA_MAX (0dBm)
  radio.stopListening();
  radio.openWritingPipe(address);
}

//*********************************************************************************************************************
// Program loop
//*********************************************************************************************************************
void loop()
{
  read_pots();
  send_and_receive_data();
  TX_batt_monitoring();
  RX_batt_monitoring();
  LED_mode();
}

//*********************************************************************************************************************
// Send and receive data
//*********************************************************************************************************************
unsigned long rf_timeout = 0;

void send_and_receive_data()
{
  if (radio.write(&rc_packet, rc_packet_size))
  {
    if (radio.available())
    {
      radio.read(&telemetry_packet, sizeof(telemetry_packet_size));
      
      rf_timeout = millis();
    }
  }
}

//*********************************************************************************************************************
// TX battery voltage monitoring
//*********************************************************************************************************************
bool tx_low_batt = 0;

void TX_batt_monitoring()
{
  tx_low_batt = analogRead(PIN_BATTERY) <= (1023 / TX_BATTERY_VOLTAGE) * TX_MONITORED_VOLTAGE;
  
  //Serial.println(tx_low_batt);
}

//*********************************************************************************************************************
// RX battery voltage monitoring
//*********************************************************************************************************************
bool rx_low_batt = 0;
bool previous_state_batt = 0;

void RX_batt_monitoring()
{
  rx_low_batt = telemetry_packet.batt_A1 <= (255 / RX_BATTERY_VOLTAGE) * RX_MONITORED_VOLTAGE;
  
  // Battery alarm lock
  if (rx_low_batt)
  {
    previous_state_batt = 1;
  }
  rx_low_batt = previous_state_batt;
  
  //Serial.println(telemetry_packet.batt_A1);
}

//*********************************************************************************************************************
// LED blink mode
//*********************************************************************************************************************
void LED_mode()
{
  if (millis() - rf_timeout > 1000) // If we lose RF data for 1 second, the LED blink at 0.1s interval
  {
    blink(PIN_LED, 100);
  }
  else if (rx_low_batt) // If the RX battery is low, the LED blink at 0.3s interval
  {
    blink(PIN_LED, 300);
  }
  else if (tx_low_batt) // If the TX battery is low, the LED blink at 0.5s interval
  {
    blink(PIN_LED, 500);
  }
  else
  {
    digitalWrite(PIN_LED, HIGH); // Normal mode, LED is lit
  }
}

//*********************************************************************************************************************
// LED blink function
//*********************************************************************************************************************
unsigned long led_time = 0;
bool led_state;

void blink(uint8_t pin, uint16_t interval)
{
  if (millis() - led_time > interval)
  {
    led_time = millis();
    
    led_state = !led_state;
    
    digitalWrite(pin, led_state);
  }
}

//*********************************************************************************************************************
// This function will write a 2 byte integer to the eeprom at the specified address and address + 1
//*********************************************************************************************************************
void EEPROMWriteInt(int p_address, int p_value)
{
  byte lowByte = p_value % 256;
  byte highByte = p_value / 256;
  EEPROM.write(p_address, lowByte);
  EEPROM.write(p_address + 1, highByte);
}

//*********************************************************************************************************************
// This function will read a 2 byte integer from the eeprom at the specified address and address + 1
//*********************************************************************************************************************
unsigned int EEPROMReadInt(int p_address)
{
  byte lowByte = EEPROM.read(p_address);
  byte highByte = EEPROM.read(p_address + 1);
  return lowByte + highByte * 256;
}
 
