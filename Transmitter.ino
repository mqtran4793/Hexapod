/*
 * Hexapod Controller Transmitter
 * Module // Arduino UNO, NANO
 * NRF24L01+
     GND    ->   GND
     Vcc    ->   3.3V
     CE     ->   D9
     CSN    ->   D10
     CLK    ->   D13
     MOSI   ->   D11
     MISO   ->   D12
 * NRF24L01+
 * 
*/

#include <SPI.h>
#include <RF24.h>
#include <nRF24l01.h>

//const byte nodeAddress = 0xE8E8F0F0E1LL; //Remember that this code should be the same for the receiver
const byte nodeAddress[5] = {'N', 'O', 'D', 'E', '1'};
const int right_Pot       = A7;
const int left_Pot        = A6;
const int buzzer          = 2;
const int RED             = 3;
const int YELLOW          = 4;
const int GREEN           = 5;
const int button_1        = 6;
const int button_2        = 8;
const int button_3        = 7;
const int right_Button    = A5;
const int left_Button     = A2;
int defaultSpeed = 30;
int defaultMode = 1;
int defaultHeight = -50;

RF24 radio(9, 10);

// The sizeof this struct should not exceed 32 bytes
struct transmitter
{
  // Right joystick
  uint8_t right_X;
  uint8_t right_Y;
  uint8_t right_Button_Val;
  // Left joystick
  uint8_t left_X;
  uint8_t left_Y;
  uint8_t left_Button_Val;
  // Potentiometer 1
  uint8_t right_Pot_Val;
  // Potentiometer 2
  uint8_t left_Pot_Val;
  // Buttons
  uint8_t button_Val_1;
  uint8_t button_Val_2;
  uint8_t button_Val_3;
};
transmitter data;

struct volt
{
  int voltage;
};
volt data1;

void buzz(int numb)
{
  switch(numb)
  {
    case 1:
      tone(buzzer, 3000);
      delay(1000);
      noTone(buzzer);
      delay(100);
      tone(buzzer, 3000);
      delay(100);
      noTone(buzzer);
      delay(100);
      tone(buzzer, 3000);
      delay(100);
      noTone(buzzer);
      break;
    case 2:
      tone(buzzer, 3000);
      delay(100);
      noTone(buzzer);
      delay(2100);
      break;
    case 3:
      tone(buzzer, 3000);
      delay(100);
      noTone(buzzer);
      delay(100);
    default:
      break;
  }
}

//int readVcc() {
//  int result;
//  // Read 1.1V reference against AVcc
//  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
//  delay(2); // Wait for Vref to settle
//  ADCSRA |= _BV(ADSC); // Convert
//  while (bit_is_set(ADCSRA, ADSC));
//  result = ADCL;
//  result |= ADCH << 8;
//  result = 1126400L / result; // Back-calculate AVcc in mV
//  return result;
//}

// Returns actual value of Vcc (x 100)
int getBattVolts()
{
  const long InternalReferenceVoltage = 1112;  // Adjust this value to your boards specific internal BG voltage x1000
//const long InternalReferenceVoltage = 1080;
    // set up for batt voltage measurement
    // REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc external reference
    // MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)         -Selects channel 14, bandgap voltage, to measure
    ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);  
    delay(25);  // Let mux settle a little to get a more stable A/D conversion
  // Start a conversion 
  ADCSRA |= _BV( ADSC );
  // Wait for it to complete
  while( ( (ADCSRA & (1<<ADSC)) != 0 ) );
  // Scale the value
  // 100L is my fudge factor to match my multimeter R2
  int result = ((((InternalReferenceVoltage * 1024L) / ADC) + 5L) / 10L)+35; 
  return result;
}
void setup()
{
  radio.begin();
  radio.setChannel(126);
  radio.setRetries(0, 15);
  radio.openWritingPipe(nodeAddress);
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.flush_tx();
  radio.stopListening();

  pinMode(buzzer, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(button_1, INPUT);
  pinMode(button_2, INPUT);
  pinMode(button_3, INPUT);
  pinMode(right_Button, OUTPUT);
  pinMode(left_Button, OUTPUT);

  // Joystick values
  digitalWrite(right_Button, HIGH);
  digitalWrite(left_Button, HIGH);
  // Button values
  data.button_Val_1 = 0;
  data.button_Val_2 = 0;
  data.button_Val_3 = 0;
  
//    // set up for batt voltage measurement
//    // REFS1 REFS0          --> 0 1, AVcc internal ref. -Selects AVcc external reference
//    // MUX3 MUX2 MUX1 MUX0  --> 1110 1.1V (VBG)         -Selects channel 14, bandgap voltage, to measure
//    ADMUX = (0<<REFS1) | (0<<REFS0) | (0<<ADLAR) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);  
//    delay(100);  // Let mux settle a little to get a more stable A/D conversion

    // Initialization completed
    buzz(1);
}

void loop()
{
  // Monitoring battery voltage
  data1.voltage = getBattVolts();
  if (data1.voltage > 450)
  {
    digitalWrite(GREEN, HIGH);
    digitalWrite(YELLOW, LOW);
    digitalWrite(RED, LOW);
  }
  else if (data1.voltage >= 350 && data1.voltage < 400)
  {
    digitalWrite(YELLOW, HIGH);
    digitalWrite(GREEN, LOW);
    digitalWrite(RED, LOW);
  }
  else if (data1.voltage < 350)
  {
    digitalWrite(RED, HIGH);
    digitalWrite(GREEN, LOW);
    digitalWrite(YELLOW, LOW);
  }
  
  data.right_Button_Val = digitalRead(right_Button);
  
  data.left_Button_Val = digitalRead(left_Button);
  data.button_Val_1 = digitalRead(button_1);
  data.button_Val_2 = digitalRead(button_2);
  data.button_Val_3 = digitalRead(button_3);

  data.right_X = map(analogRead(A3), 0, 1023, 0, 255);
  data.right_Y = map(analogRead(A4), 0, 1023, 0, 255);
  data.left_X = map(analogRead(A0), 0, 1023, 0, 255);
  data.left_Y = map(analogRead(A1), 0, 1023, 0, 255);
  data.right_Pot_Val = map(analogRead(A7), 0, 1023, 0, 255);
  data.left_Pot_Val = map(analogRead(A6), 0, 1023, 0, 255);

//  if(data.right_Button_Val == LOW)
//  {
//    Serial.println("Right joystick pressed");
////    buzz(2);
//  }
//  else if(data.left_Button_Val == LOW)
//  {
//    Serial.println("Left joystick pressed");
////    buzz(2);
//  }
//  else if(data.button_Val_1 == HIGH)
//  {
//    Serial.println("Button 1 pressed");
////    buzz(2);
//  }
//  else if(data.button_Val_2 == HIGH)
//  {
//    Serial.println("Button 2 pressed");
////    buzz(2);
//  }
//  else if(data.button_Val_3 == HIGH)
//  {
//    Serial.println("Button 3 pressed");
////    buzz(2);
//  }

  // Transmitting data to Hexapod
  radio.write(&data, sizeof(data));
  delay(500);

}
