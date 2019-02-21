/*
  Module // Arduino UNO, NANO
  
  GND    ->   GND
  Vcc    ->   3.3V
  CE     ->   D9
  CSN    ->   D10
  CLK    ->   D13
  MOSI   ->   D11
  MISO   ->   D12
*/

#include <RF24.h>

const uint64_t radio_pipe = 0xE8E8F0F0E1LL; //Remember that this code should be the same for the receiver
RF24 radio(9, 10);

// The sizeof this struct should not exceed 32 bytes
struct Data
{
  byte Right_x;
  byte Right_y;
  unsigned int Right_button = 1;
  byte Left_x;
  byte Left_y;
  unsigned int Left_button = 1;
};
Data data;
void setup()
{
//  Serial.begin(9600);
  radio.begin();
  radio.setChannel(108);
//  radio.setRetries(0, 15);
  radio.openWritingPipe(radio_pipe);
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
//  radio.setDataRate(RF24_2MBPS);
  radio.stopListening();

  data.Left_x         = 0;
  data.Left_y         = 0;
  data.Left_button    = 0;
  data.Right_x        = 0;
  data.Right_y        = 0;
  data.Right_button   = 0;
}

//void loop()
//{
//  const char text[] = "NRFTest";
//  radio.write(&text, sizeof(text));
//  delay(500);
//}

void loop()
{
//  if (analogRead(A2) == 0)
//  {
//    data.Right_button = 0;
//  }
//  else
//  {
//    data.Right_button = 1;
//  }
//
//  if (analogRead(A5) == 0)
//  {
//    data.Left_button = 0;
//  }
//  else
//  {
//    data.Left_button = 1;
//  }
  data.Right_x = map(analogRead(A0), 0, 1024, 0, 255);
  data.Right_y = map(analogRead(A1), 0, 1024, 0, 255);
  data.Left_x = map(analogRead(A3), 0, 1024, 0, 255);
  data.Left_y = map(analogRead(A4), 0, 1024, 0, 255);
//  data.Right_button = map(analogRead(A2), 0, 1024, 0, 255);
//  data.Left_button = map(analogRead(A5), 0, 1024, 0, 255);
  Serial.println("Left button: ");
  Serial.println(analogRead(A5), DEC);
  Serial.println("Right button: ");
  Serial.println(analogRead(A2), DEC);
  radio.write(&data, sizeof(data));
  delay(1000);
}
