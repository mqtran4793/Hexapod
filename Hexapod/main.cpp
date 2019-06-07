#include <iostream>
#include <RF24/RF24.h>
#include <string>
#include "InverseKinematics.h"

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <termios.h>

#include <wiringPi.h>
using namespace std;

RF24 radio(RPI_V2_GPIO_P1_15, RPI_V2_GPIO_P1_24, BCM2835_SPI_SPEED_8MHZ);
const uint8_t nodeAddress[5] = {'N', 'O', 'D', 'E', '1'};
//const uint8_t nodeAddress = 0xE8E8F0F0E1LL;
const int LED = 4;

struct receiver
{
	uint8_t right_X;
	uint8_t right_Y;
	uint8_t right_Button;
	// Left joystick
	uint8_t left_X;
	uint8_t left_Y;
	uint8_t left_Button;
	// Potentiometer 1
	uint8_t right_Pot;
	// Potentiometer 2
	uint8_t left_Pot;
	// Buttons
	uint8_t button_1;
	uint8_t button_2;
	uint8_t button_3;
};
receiver data;

int right_X_Val, right_Y_Val, right_Button_Val;
int left_X_Val, left_Y_Val, left_Button_Val;
int cam_Top_Val, cam_Base_Val;
int button_1_Val, button_2_Val, button_3_Val;
int lastState = 0;
int movement_Mode = 1;
int config_Mode = 1;
int height = -50;
int speed = 30;

void setup(void)
{
	radio.begin();
	radio.setChannel(126);
	radio.setRetries(0, 15);
	radio.openReadingPipe(1, nodeAddress);
	radio.setAutoAck(false);
	radio.setPALevel(RF24_PA_HIGH);
	radio.setDataRate(RF24_250KBPS);
	radio.flush_rx();
	radio.startListening();
}


LegAngles Leg;
InverseKinematics Hexapod;
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
}

int maestroGetPosition(int fd, unsigned char channel)
{
	unsigned char command[2] = {0x90, channel};
	if(write(fd, command, sizeof(command)) == -1)
	{
		perror("error writing");
		return -1;
	}

	unsigned char response[2];
	if(read(fd,response,2) != 2)
	{
		perror("error reading");
		return -1;
	}

	return response[0] + 256*response[1];
}

int maestroSetTarget(int fd, unsigned char channel, unsigned short target)
{
	unsigned char command[4] = {0x84, channel, target & 0x7F, target >> 7 & 0x7F};
	if (write(fd, command, sizeof(command)) == -1)
	{
		perror("error writing");
		return -1;
	}
	return 0;
}

int maestroSetSpeed(int fd, unsigned char channel, unsigned short speed)
{
	unsigned char command[4] = {0x87, channel, speed & 0x7F, speed >> 7 & 0x7F};
	if (write(fd, command, sizeof(command)) == -1)
	{
		perror("error writing");
		return -1;
	}
	return 0;
}

int maestroSetAcce(int fd, unsigned char channel, unsigned short acce)
{
	unsigned char command[4] = {0x89, channel, acce & 0x7F, acce >> 7 & 0x7F};
	if (write(fd, command, sizeof(command)) == -1)
	{
		perror("error writing");
		return -1;
	}
	return 0;
}

void delay(int number_of_miliseconds)
{
    // Converting time into milli_seconds
    int seconds = 1000 * number_of_miliseconds;

    // String start time
    clock_t start_time = clock();

    // Looping till required time is not achieved
    while (clock() < start_time + seconds);
}

void MoveServo(int fd, int legNumb)
{
	float coxa, femur, tibia;
	switch (legNumb)
	{
		case 0:
			tibia = map(Leg.Tibia[legNumb], 45, 180, 848, 2384);
			femur = map(Leg.Femur[legNumb], 0, 180, 384, 2400);
			coxa = map(Leg.Coxa[legNumb], 45, 135, 972, 1988);
			maestroSetTarget(fd, 0, tibia*4);
			maestroSetTarget(fd, 1, femur*4);
			maestroSetTarget(fd, 2, coxa*4);
			break;
		case 1:
			tibia = map(Leg.Tibia[legNumb], 45, 180, 848, 2384);
			femur = map(Leg.Femur[legNumb], 0, 180, 384, 2400);
			coxa = map(Leg.Coxa[legNumb], 45, 135, 972, 1988);
			maestroSetTarget(fd, 3, tibia*4);
			maestroSetTarget(fd, 4, femur*4);
			maestroSetTarget(fd, 5, coxa*4);
			break;
		case 2:
			tibia = map(Leg.Tibia[legNumb], 45, 180, 848, 2384);
			femur = map(Leg.Femur[legNumb], 0, 180, 384, 2400);
			coxa = map(Leg.Coxa[legNumb], 45, 135, 972, 1988);
			maestroSetTarget(fd, 6, tibia*4);
			maestroSetTarget(fd, 7, femur*4);
			maestroSetTarget(fd, 8, coxa*4);
			break;
		case 3:
			tibia = map(Leg.Tibia[legNumb], 45, 180, 848, 2384);
			femur = map(Leg.Femur[legNumb], 0, 180, 384, 2400);
			coxa = map(Leg.Coxa[legNumb], 45, 135, 972, 1988);
			maestroSetTarget(fd, 9, tibia*4);
			maestroSetTarget(fd, 10, femur*4);
			maestroSetTarget(fd, 11, coxa*4);
			break;
		case 4:
			tibia = map(Leg.Tibia[legNumb], 45, 180, 848, 2384);
			femur = map(Leg.Femur[legNumb], 0, 180, 384, 2400);
			coxa = map(Leg.Coxa[legNumb], 45, 135, 972, 1988);
			maestroSetTarget(fd, 12, tibia*4);
			maestroSetTarget(fd, 13, femur*4);
			maestroSetTarget(fd, 14, coxa*4);
			break;
		case 5:
			tibia = map(Leg.Tibia[legNumb], 45, 180, 848, 2384);
			femur = map(Leg.Femur[legNumb], 0, 180, 384, 2400);
			coxa = map(Leg.Coxa[legNumb], 45, 135, 972, 1988);
			maestroSetTarget(fd, 15, tibia*4);
			maestroSetTarget(fd, 16, femur*4);
			maestroSetTarget(fd, 17, coxa*4);
			break;
		default:
		cout << "Invalid leg number!!!" << endl;
	}
}

void initHex(int fd, int Pos_x, int Pos_y, int Pos_z, float Rot_x, float Rot_y, float Rot_z)
{
	for (int legNumb = 0; legNumb < 6; legNumb++)
	{
		Leg = Hexapod.LegIK(legNumb, Pos_x, Pos_y, Pos_z, Rot_x, Rot_y, Rot_z);
	}

	for (int legNumb = 0; legNumb < 6; legNumb++)
	{
		MoveServo(fd, legNumb);
	}

	maestroSetTarget(fd, 18, 4800);
	maestroSetTarget(fd, 19, 6464);
}

void MoveLeg(int fd, int legNumb, int Pos_x, int Pos_y, int Pos_z, float Rot_x, float Rot_y, float Rot_z)
{
	Leg = Hexapod.LegIK(legNumb, Pos_x, Pos_y, Pos_z, Rot_x, Rot_y, Rot_z);
	MoveServo(fd, legNumb);
}

void MoveForward(int fd, int h)
{
	// lift
	MoveLeg(fd, 0, 3, -40, h-25, 0, 0, 0);
	MoveLeg(fd, 4, 3, -40, h-25, 0, 0, 0);
	MoveLeg(fd, 2, 3, -40, h-25, 0, 0, 0);

	MoveLeg(fd, 0, 3, 0, h-50, 0, 0, 0);
	MoveLeg(fd, 4, 3, 0, h-50, 0, 0, 0);
	MoveLeg(fd, 2, 3, 0, h-50, 0, 0, 0);

	MoveLeg(fd, 5, 3, 30, h, 0, 0, 0);
	MoveLeg(fd, 1, 3, 30, h, 0, 0, 0);
	MoveLeg(fd, 3, 3, 30, h, 0, 0, 0);

	MoveLeg(fd, 5, 3, 20, h, 0, 0, 0);
	MoveLeg(fd, 1, 3, 20, h, 0, 0, 0);
	MoveLeg(fd, 3, 3, 20, h, 0, 0, 0);

	MoveLeg(fd, 5, 3, 10, h, 0, 0, 0);
	MoveLeg(fd, 1, 3, 10, h, 0, 0, 0);
	MoveLeg(fd, 3, 3, 10, h, 0, 0, 0);

	MoveLeg(fd, 5, 3, 0, h, 0, 0, 0);
	MoveLeg(fd, 1, 3, 0, h, 0, 0, 0);
	MoveLeg(fd, 3, 3, 0, h, 0, 0, 0);

	MoveLeg(fd, 0, 3, 40, h-25, 0, 0, 0);
	MoveLeg(fd, 4, 3, 40, h-25, 0, 0, 0);
	MoveLeg(fd, 2, 3, 40, h-25, 0, 0, 0);

	// back down
	MoveLeg(fd, 5, 3, -10, h, 0, 0, 0);
	MoveLeg(fd, 1, 3, -10, h, 0, 0, 0);
	MoveLeg(fd, 3, 3, -10, h, 0, 0, 0);

	MoveLeg(fd, 5, 3, -20, h, 0, 0, 0);
	MoveLeg(fd, 1, 3, -20, h, 0, 0, 0);
	MoveLeg(fd, 3, 3, -20, h, 0, 0, 0);

	MoveLeg(fd, 5, 3, -30, h, 0, 0, 0);
	MoveLeg(fd, 1, 3, -30, h, 0, 0, 0);
	MoveLeg(fd, 3, 3, -30, h, 0, 0, 0);

	MoveLeg(fd, 5, 3, -40, h, 0, 0, 0);
	MoveLeg(fd, 1, 3, -40, h, 0, 0, 0);
	MoveLeg(fd, 3, 3, -40, h, 0, 0, 0);

	// forward down
	MoveLeg(fd, 0, 3, 40, h, 0, 0, 0);
	MoveLeg(fd, 4, 3, 40, h, 0, 0, 0);
	MoveLeg(fd, 2, 3, 40, h, 0, 0, 0);

	// lift
	MoveLeg(fd, 5, 3, -40, h-25, 0, 0, 0);
	MoveLeg(fd, 1, 3, -40, h-25, 0, 0, 0);
	MoveLeg(fd, 3, 3, -40, h-25, 0, 0, 0);

	MoveLeg(fd, 5, 3, 0, h-50, 0, 0, 0);
	MoveLeg(fd, 1, 3, 0, h-50, 0, 0, 0);
	MoveLeg(fd, 3, 3, 0, h-50, 0, 0, 0);

	MoveLeg(fd, 0, 3, 30, h, 0, 0, 0);
	MoveLeg(fd, 4, 3, 30, h, 0, 0, 0);
	MoveLeg(fd, 2, 3, 30, h, 0, 0, 0);

	MoveLeg(fd, 0, 3, 20, h, 0, 0, 0);
	MoveLeg(fd, 4, 3, 20, h, 0, 0, 0);
	MoveLeg(fd, 2, 3, 20, h, 0, 0, 0);

	MoveLeg(fd, 0, 3, 10, h, 0, 0, 0);
	MoveLeg(fd, 4, 3, 10, h, 0, 0, 0);
	MoveLeg(fd, 2, 3, 10, h, 0, 0, 0);

	MoveLeg(fd, 0, 3, 0, h, 0, 0, 0);
	MoveLeg(fd, 4, 3, 0, h, 0, 0, 0);
	MoveLeg(fd, 2, 3, 0, h, 0, 0, 0);

	MoveLeg(fd, 5, 3, 40, h-25, 0, 0, 0);
	MoveLeg(fd, 1, 3, 40, h-25, 0, 0, 0);
	MoveLeg(fd, 3, 3, 40, h-25, 0, 0, 0);

	// back down
	MoveLeg(fd, 0, 3, -10, h, 0, 0, 0);
	MoveLeg(fd, 4, 3, -10, h, 0, 0, 0);
	MoveLeg(fd, 2, 3, -10, h, 0, 0, 0);

	MoveLeg(fd, 0, 3, -20, h, 0, 0, 0);
	MoveLeg(fd, 4, 3, -20, h, 0, 0, 0);
	MoveLeg(fd, 2, 3, -20, h, 0, 0, 0);

	MoveLeg(fd, 0, 3, -30, h, 0, 0, 0);
	MoveLeg(fd, 4, 3, -30, h, 0, 0, 0);
	MoveLeg(fd, 2, 3, -30, h, 0, 0, 0);

	MoveLeg(fd, 0, 3, -40, h, 0, 0, 0);
	MoveLeg(fd, 4, 3, -40, h, 0, 0, 0);
	MoveLeg(fd, 2, 3, -40, h, 0, 0, 0);

	// forward down
	MoveLeg(fd, 5, 3, 40, h, 0, 0, 0);
	MoveLeg(fd, 1, 3, 40, h, 0, 0, 0);
	MoveLeg(fd, 3, 3, 40, h, 0, 0, 0);
}

void MoveBackward(int fd, int h)
{
	// lift
	MoveLeg(fd, 0, 0, 40, h-25, 0, 0, 0);
	MoveLeg(fd, 4, 0, 40, h-25, 0, 0, 0);
	MoveLeg(fd, 2, 0, 40, h-25, 0, 0, 0);

	MoveLeg(fd, 0, 0, 0, h-50, 0, 0, 0);
	MoveLeg(fd, 4, 0, 0, h-50, 0, 0, 0);
	MoveLeg(fd, 2, 0, 0, h-50, 0, 0, 0);

	MoveLeg(fd, 5, 0, -30, h, 0, 0, 0);
	MoveLeg(fd, 1, 0, -30, h, 0, 0, 0);
	MoveLeg(fd, 3, 0, -30, h, 0, 0, 0);

	MoveLeg(fd, 5, 0, -20, h, 0, 0, 0);
	MoveLeg(fd, 1, 0, -20, h, 0, 0, 0);
	MoveLeg(fd, 3, 0, -20, h, 0, 0, 0);

	MoveLeg(fd, 5, 0, -10, h, 0, 0, 0);
	MoveLeg(fd, 1, 0, -10, h, 0, 0, 0);
	MoveLeg(fd, 3, 0, -10, h, 0, 0, 0);

	MoveLeg(fd, 5, 0, 0, h, 0, 0, 0);
	MoveLeg(fd, 1, 0, 0, h, 0, 0, 0);
	MoveLeg(fd, 3, 0, 0, h, 0, 0, 0);

	MoveLeg(fd, 0, 0, -40, h-25, 0, 0, 0);
	MoveLeg(fd, 4, 0, -40, h-25, 0, 0, 0);
	MoveLeg(fd, 2, 0, -40, h-25, 0, 0, 0);

	// back down
	MoveLeg(fd, 5, 0, 10, h, 0, 0, 0);
	MoveLeg(fd, 1, 0, 10, h, 0, 0, 0);
	MoveLeg(fd, 3, 0, 10, h, 0, 0, 0);

	MoveLeg(fd, 5, 0, 20, h, 0, 0, 0);
	MoveLeg(fd, 1, 0, 20, h, 0, 0, 0);
	MoveLeg(fd, 3, 0, 20, h, 0, 0, 0);

	MoveLeg(fd, 5, 0, 30, h, 0, 0, 0);
	MoveLeg(fd, 1, 0, 30, h, 0, 0, 0);
	MoveLeg(fd, 3, 0, 30, h, 0, 0, 0);

	MoveLeg(fd, 5, 0, 40, h, 0, 0, 0);
	MoveLeg(fd, 1, 0, 40, h, 0, 0, 0);
	MoveLeg(fd, 3, 0, 40, h, 0, 0, 0);

	// forward down
	MoveLeg(fd, 0, 0, -40, h, 0, 0, 0);
	MoveLeg(fd, 4, 0, -40, h, 0, 0, 0);
	MoveLeg(fd, 2, 0, -40, h, 0, 0, 0);

	// lift
	MoveLeg(fd, 5, 0, 40, h-25, 0, 0, 0);
	MoveLeg(fd, 1, 0, 40, h-25, 0, 0, 0);
	MoveLeg(fd, 3, 0, 40, h-25, 0, 0, 0);

	MoveLeg(fd, 5, 0, 0, h-50, 0, 0, 0);
	MoveLeg(fd, 1, 0, 0, h-50, 0, 0, 0);
	MoveLeg(fd, 3, 0, 0, h-50, 0, 0, 0);

	MoveLeg(fd, 0, 0, -30, h, 0, 0, 0);
	MoveLeg(fd, 4, 0, -30, h, 0, 0, 0);
	MoveLeg(fd, 2, 0, -30, h, 0, 0, 0);

	MoveLeg(fd, 0, 0, -20, h, 0, 0, 0);
	MoveLeg(fd, 4, 0, -20, h, 0, 0, 0);
	MoveLeg(fd, 2, 0, -20, h, 0, 0, 0);

	MoveLeg(fd, 0, 0, -10, h, 0, 0, 0);
	MoveLeg(fd, 4, 0, -10, h, 0, 0, 0);
	MoveLeg(fd, 2, 0, -10, h, 0, 0, 0);

	MoveLeg(fd, 0, 0, 0, h, 0, 0, 0);
	MoveLeg(fd, 4, 0, 0, h, 0, 0, 0);
	MoveLeg(fd, 2, 0, 0, h, 0, 0, 0);

	MoveLeg(fd, 5, 0, -40, h-25, 0, 0, 0);
	MoveLeg(fd, 1, 0, -40, h-25, 0, 0, 0);
	MoveLeg(fd, 3, 0, -40, h-25, 0, 0, 0);

	// back down
	MoveLeg(fd, 0, 0, 10, h, 0, 0, 0);
	MoveLeg(fd, 4, 0, 10, h, 0, 0, 0);
	MoveLeg(fd, 2, 0, 10, h, 0, 0, 0);

	MoveLeg(fd, 0, 0, 20, h, 0, 0, 0);
	MoveLeg(fd, 4, 0, 20, h, 0, 0, 0);
	MoveLeg(fd, 2, 0, 20, h, 0, 0, 0);

	MoveLeg(fd, 0, 0, 30, h, 0, 0, 0);
	MoveLeg(fd, 4, 0, 30, h, 0, 0, 0);
	MoveLeg(fd, 2, 0, 30, h, 0, 0, 0);

	MoveLeg(fd, 0, 0, 40, h, 0, 0, 0);
	MoveLeg(fd, 4, 0, 40, h, 0, 0, 0);
	MoveLeg(fd, 2, 0, 40, h, 0, 0, 0);

	// forward down
	MoveLeg(fd, 5, 0, -40, h, 0, 0, 0);
	MoveLeg(fd, 1, 0, -40, h, 0, 0, 0);
	MoveLeg(fd, 3, 0, -40, h, 0, 0, 0);
}

void MoveLeft(int fd, int h)
{
	// lift
	MoveLeg(fd, 0, 40, 0, h-25, 0, 0, 0);
	MoveLeg(fd, 4, 40, 0, h-25, 0, 0, 0);
	MoveLeg(fd, 2, 40, 0, h-25, 0, 0, 0);

	MoveLeg(fd, 0, 0, 0, h-50, 0, 0, 0);
	MoveLeg(fd, 4, 0, 0, h-50, 0, 0, 0);
	MoveLeg(fd, 2, 0, 0, h-50, 0, 0, 0);

	MoveLeg(fd, 5, -30, 0, h, 0, 0, 0);
	MoveLeg(fd, 1, -30, 0, h, 0, 0, 0);
	MoveLeg(fd, 3, -30, 0, h, 0, 0, 0);

	MoveLeg(fd, 5, -20, 0, h, 0, 0, 0);
	MoveLeg(fd, 1, -20, 0, h, 0, 0, 0);
	MoveLeg(fd, 3, -20, 0, h, 0, 0, 0);

	MoveLeg(fd, 5, -10, 0, h, 0, 0, 0);
	MoveLeg(fd, 1, -10, 0, h, 0, 0, 0);
	MoveLeg(fd, 3, -10, 0, h, 0, 0, 0);

	MoveLeg(fd, 5, 0, 0, h, 0, 0, 0);
	MoveLeg(fd, 1, 0, 0, h, 0, 0, 0);
	MoveLeg(fd, 3, 0, 0, h, 0, 0, 0);

	MoveLeg(fd, 0, -40, 0, h-25, 0, 0, 0);
	MoveLeg(fd, 4, -40, 0, h-25, 0, 0, 0);
	MoveLeg(fd, 2, -40, 0, h-25, 0, 0, 0);

	// back
	MoveLeg(fd, 5, 10, 0, h, 0, 0, 0);
	MoveLeg(fd, 1, 10, 0, h, 0, 0, 0);
	MoveLeg(fd, 3, 10, 0, h, 0, 0, 0);

	MoveLeg(fd, 5, 20, 0, h, 0, 0, 0);
	MoveLeg(fd, 1, 20, 0, h, 0, 0, 0);
	MoveLeg(fd, 3, 20, 0, h, 0, 0, 0);

	MoveLeg(fd, 5, 30, 0, h, 0, 0, 0);
	MoveLeg(fd, 1, 30, 0, h, 0, 0, 0);
	MoveLeg(fd, 3, 30, 0, h, 0, 0, 0);

	MoveLeg(fd, 5, 40, 0, h, 0, 0, 0);
	MoveLeg(fd, 1, 40, 0, h, 0, 0, 0);
	MoveLeg(fd, 3, 40, 0, h, 0, 0, 0);

	// forward down
	MoveLeg(fd, 0, -40, 0, h, 0, 0, 0);
	MoveLeg(fd, 4, -40, 0, h, 0, 0, 0);
	MoveLeg(fd, 2, -40, 0, h, 0, 0, 0);

	// lift
	MoveLeg(fd, 5, 40, 0, h-25, 0, 0, 0);
	MoveLeg(fd, 1, 40, 0, h-25, 0, 0, 0);
	MoveLeg(fd, 3, 40, 0, h-25, 0, 0, 0);

	MoveLeg(fd, 5, 0, 0, h-50, 0, 0, 0);
	MoveLeg(fd, 1, 0, 0, h-50, 0, 0, 0);
	MoveLeg(fd, 3, 0, 0, h-50, 0, 0, 0);

	MoveLeg(fd, 0, -30, 0, h, 0, 0, 0);
	MoveLeg(fd, 4, -30, 0, h, 0, 0, 0);
	MoveLeg(fd, 2, -30, 0, h, 0, 0, 0);

	MoveLeg(fd, 0, -20, 0, h, 0, 0, 0);
	MoveLeg(fd, 4, -20, 0, h, 0, 0, 0);
	MoveLeg(fd, 2, -20, 0, h, 0, 0, 0);

	MoveLeg(fd, 0, -10, 0, h, 0, 0, 0);
	MoveLeg(fd, 4, -10, 0, h, 0, 0, 0);
	MoveLeg(fd, 2, -10, 0, h, 0, 0, 0);

	MoveLeg(fd, 0, 0, 0, h, 0, 0, 0);
	MoveLeg(fd, 4, 0, 0, h, 0, 0, 0);
	MoveLeg(fd, 2, 0, 0, h, 0, 0, 0);

	MoveLeg(fd, 5, -40, 0, h-25, 0, 0, 0);
	MoveLeg(fd, 1, -40, 0, h-25, 0, 0, 0);
	MoveLeg(fd, 3, -40, 0, h-25, 0, 0, 0);

	// back
	MoveLeg(fd, 0, 10, 0, h, 0, 0, 0);
	MoveLeg(fd, 4, 10, 0, h, 0, 0, 0);
	MoveLeg(fd, 2, 10, 0, h, 0, 0, 0);

	MoveLeg(fd, 0, 20, 0, h, 0, 0, 0);
	MoveLeg(fd, 4, 20, 0, h, 0, 0, 0);
	MoveLeg(fd, 2, 20, 0, h, 0, 0, 0);

	MoveLeg(fd, 0, 30, 0, h, 0, 0, 0);
	MoveLeg(fd, 4, 30, 0, h, 0, 0, 0);
	MoveLeg(fd, 2, 30, 0, h, 0, 0, 0);

	MoveLeg(fd, 0, 40, 0, h, 0, 0, 0);
	MoveLeg(fd, 4, 40, 0, h, 0, 0, 0);
	MoveLeg(fd, 2, 40, 0, h, 0, 0, 0);

	// forward down
	MoveLeg(fd, 5, -40, 0, h, 0, 0, 0);
	MoveLeg(fd, 1, -40, 0, h, 0, 0, 0);
	MoveLeg(fd, 3, -40, 0, h, 0, 0, 0);
}

void MoveRight(int fd, int h)
{
	MoveLeg(fd, 0, -40, 0, h-25, 0, 0, 0);
	MoveLeg(fd, 4, -40, 0, h-25, 0, 0, 0);
	MoveLeg(fd, 2, -40, 0, h-25, 0, 0, 0);

	MoveLeg(fd, 0, 0, 0, h-50, 0, 0, 0);
	MoveLeg(fd, 4, 0, 0, h-50, 0, 0, 0);
	MoveLeg(fd, 2, 0, 0, h-50, 0, 0, 0);

	MoveLeg(fd, 5, 30, 0, h, 0, 0, 0);
	MoveLeg(fd, 1, 30, 0, h, 0, 0, 0);
	MoveLeg(fd, 3, 30, 0, h, 0, 0, 0);

	MoveLeg(fd, 5, 20, 0, h, 0, 0, 0);
	MoveLeg(fd, 1, 20, 0, h, 0, 0, 0);
	MoveLeg(fd, 3, 20, 0, h, 0, 0, 0);

	MoveLeg(fd, 5, 10, 0, h, 0, 0, 0);
	MoveLeg(fd, 1, 10, 0, h, 0, 0, 0);
	MoveLeg(fd, 3, 10, 0, h, 0, 0, 0);

	MoveLeg(fd, 5, 0, 0, h, 0, 0, 0);
	MoveLeg(fd, 1, 0, 0, h, 0, 0, 0);
	MoveLeg(fd, 3, 0, 0, h, 0, 0, 0);

	MoveLeg(fd, 0, 40, 0, h-25, 0, 0, 0);
	MoveLeg(fd, 4, 40, 0, h-25, 0, 0, 0);
	MoveLeg(fd, 2, 40, 0, h-25, 0, 0, 0);

	// back
	MoveLeg(fd, 5, -10, 0, h, 0, 0, 0);
	MoveLeg(fd, 1, -10, 0, h, 0, 0, 0);
	MoveLeg(fd, 3, -10, 0, h, 0, 0, 0);

	MoveLeg(fd, 5, -20, 0, h, 0, 0, 0);
	MoveLeg(fd, 1, -20, 0, h, 0, 0, 0);
	MoveLeg(fd, 3, -20, 0, h, 0, 0, 0);

	MoveLeg(fd, 5, -30, 0, h, 0, 0, 0);
	MoveLeg(fd, 1, -30, 0, h, 0, 0, 0);
	MoveLeg(fd, 3, -30, 0, h, 0, 0, 0);

	MoveLeg(fd, 5, -40, 0, h, 0, 0, 0);
	MoveLeg(fd, 1, -40, 0, h, 0, 0, 0);
	MoveLeg(fd, 3, -40, 0, h, 0, 0, 0);

	// forward down
	MoveLeg(fd, 0, 40, 0, h, 0, 0, 0);
	MoveLeg(fd, 4, 40, 0, h, 0, 0, 0);
	MoveLeg(fd, 2, 40, 0, h, 0, 0, 0);

	// lift
	MoveLeg(fd, 5, -40, 0, h-25, 0, 0, 0);
	MoveLeg(fd, 1, -40, 0, h-25, 0, 0, 0);
	MoveLeg(fd, 3, -40, 0, h-25, 0, 0, 0);

	MoveLeg(fd, 5, 0, 0, h-50, 0, 0, 0);
	MoveLeg(fd, 1, 0, 0, h-50, 0, 0, 0);
	MoveLeg(fd, 3, 0, 0, h-50, 0, 0, 0);

	MoveLeg(fd, 0, 30, 0, h, 0, 0, 0);
	MoveLeg(fd, 4, 30, 0, h, 0, 0, 0);
	MoveLeg(fd, 2, 30, 0, h, 0, 0, 0);

	MoveLeg(fd, 0, 20, 0, h, 0, 0, 0);
	MoveLeg(fd, 4, 20, 0, h, 0, 0, 0);
	MoveLeg(fd, 2, 20, 0, h, 0, 0, 0);

	MoveLeg(fd, 0, 10, 0, h, 0, 0, 0);
	MoveLeg(fd, 4, 10, 0, h, 0, 0, 0);
	MoveLeg(fd, 2, 10, 0, h, 0, 0, 0);

	MoveLeg(fd, 0, 0, 0, h, 0, 0, 0);
	MoveLeg(fd, 4, 0, 0, h, 0, 0, 0);
	MoveLeg(fd, 2, 0, 0, h, 0, 0, 0);

	MoveLeg(fd, 5, 40, 0, h-25, 0, 0, 0);
	MoveLeg(fd, 1, 40, 0, h-25, 0, 0, 0);
	MoveLeg(fd, 3, 40, 0, h-25, 0, 0, 0);

	// back
	MoveLeg(fd, 0, -10, 0, h, 0, 0, 0);
	MoveLeg(fd, 4, -10, 0, h, 0, 0, 0);
	MoveLeg(fd, 2, -10, 0, h, 0, 0, 0);

	MoveLeg(fd, 0, -20, 0, h, 0, 0, 0);
	MoveLeg(fd, 4, -20, 0, h, 0, 0, 0);
	MoveLeg(fd, 2, -20, 0, h, 0, 0, 0);

	MoveLeg(fd, 0, -30, 0, h, 0, 0, 0);
	MoveLeg(fd, 4, -30, 0, h, 0, 0, 0);
	MoveLeg(fd, 2, -30, 0, h, 0, 0, 0);

	MoveLeg(fd, 0, -40, 0, h, 0, 0, 0);
	MoveLeg(fd, 4, -40, 0, h, 0, 0, 0);
	MoveLeg(fd, 2, -40, 0, h, 0, 0, 0);

	// forward down
	MoveLeg(fd, 5, 40, 0, h, 0, 0, 0);
	MoveLeg(fd, 1, 40, 0, h, 0, 0, 0);
	MoveLeg(fd, 3, 40, 0, h, 0, 0, 0);
}

void SwivelLeft(int fd, int h)
{
	// lift
	MoveLeg(fd, 0, 0, 0, h-20, 0, 0, -15);
	MoveLeg(fd, 4, 0, 0, h-20, 0, 0, -15);
	MoveLeg(fd, 2, 0, 0, h-20, 0, 0, -15);

	MoveLeg(fd, 0, 0, 0, h-30, 0, 0, -10);
	MoveLeg(fd, 4, 0, 0, h-30, 0, 0, -10);
	MoveLeg(fd, 2, 0, 0, h-30, 0, 0, -10);

	MoveLeg(fd, 0, 0, 0, h-50, 0, 0, -5);
	MoveLeg(fd, 4, 0, 0, h-50, 0, 0, -5);
	MoveLeg(fd, 2, 0, 0, h-50, 0, 0, -5);

	MoveLeg(fd, 0, 0, 0, h-50, 0, 0, 0);
	MoveLeg(fd, 4, 0, 0, h-50, 0, 0, 0);
	MoveLeg(fd, 2, 0, 0, h-50, 0, 0, 0);

	MoveLeg(fd, 0, 0, 0, h-30, 0, 0, 5);
	MoveLeg(fd, 4, 0, 0, h-30, 0, 0, 5);
	MoveLeg(fd, 2, 0, 0, h-30, 0, 0, 5);

	// back
	MoveLeg(fd, 5, 0, 0, h, 0, 0, 10);
	MoveLeg(fd, 1, 0, 0, h, 0, 0, 10);
	MoveLeg(fd, 3, 0, 0, h, 0, 0, 10);

	MoveLeg(fd, 5, 0, 0, h, 0, 0, 5);
	MoveLeg(fd, 1, 0, 0, h, 0, 0, 5);
	MoveLeg(fd, 3, 0, 0, h, 0, 0, 5);

	MoveLeg(fd, 5, 0, 0, h, 0, 0, 0);
	MoveLeg(fd, 1, 0, 0, h, 0, 0, 0);
	MoveLeg(fd, 3, 0, 0, h, 0, 0, 0);

	MoveLeg(fd, 5, 0, 0, h, 0, 0, -5);
	MoveLeg(fd, 1, 0, 0, h, 0, 0, -5);
	MoveLeg(fd, 3, 0, 0, h, 0, 0, -5);

	MoveLeg(fd, 5, 0, 0, h, 0, 0, -10);
	MoveLeg(fd, 1, 0, 0, h, 0, 0, -10);
	MoveLeg(fd, 3, 0, 0, h, 0, 0, -10);

	MoveLeg(fd, 5, 0, 0, h, 0, 0, -15);
	MoveLeg(fd, 1, 0, 0, h, 0, 0, -15);
	MoveLeg(fd, 3, 0, 0, h, 0, 0, -15);

	// down
	MoveLeg(fd, 0, 0, 0, h-20, 0, 0, 10);
	MoveLeg(fd, 4, 0, 0, h-20, 0, 0, 10);
	MoveLeg(fd, 2, 0, 0, h-20, 0, 0, 10);

	MoveLeg(fd, 0, 0, 0, h, 0, 0, 15);
	MoveLeg(fd, 4, 0, 0, h, 0, 0, 15);
	MoveLeg(fd, 2, 0, 0, h, 0, 0, 15);

	// lift
	MoveLeg(fd, 5, 0, 0, h, 0, 0, -15);
	MoveLeg(fd, 1, 0, 0, h, 0, 0, -15);
	MoveLeg(fd, 3, 0, 0, h, 0, 0, -15);

	MoveLeg(fd, 5, 0, 0, h-20, 0, 0, -10);
	MoveLeg(fd, 1, 0, 0, h-20, 0, 0, -10);
	MoveLeg(fd, 3, 0, 0, h-20, 0, 0, -10);

	MoveLeg(fd, 5, 0, 0, h-30, 0, 0, -5);
	MoveLeg(fd, 1, 0, 0, h-30, 0, 0, -5);
	MoveLeg(fd, 3, 0, 0, h-30, 0, 0, -5);

	MoveLeg(fd, 5, 0, 0, h-50, 0, 0, 0);
	MoveLeg(fd, 1, 0, 0, h-50, 0, 0, 0);
	MoveLeg(fd, 3, 0, 0, h-50, 0, 0, 0);

	MoveLeg(fd, 5, 0, 0, h-30, 0, 0, 5);
	MoveLeg(fd, 1, 0, 0, h-30, 0, 0, 5);
	MoveLeg(fd, 3, 0, 0, h-30, 0, 0, 5);

	// back
	MoveLeg(fd, 0, 0, 0, h, 0, 0, 10);
	MoveLeg(fd, 4, 0, 0, h, 0, 0, 10);
	MoveLeg(fd, 2, 0, 0, h, 0, 0, 10);

	MoveLeg(fd, 0, 0, 0, h, 0, 0, 5);
	MoveLeg(fd, 4, 0, 0, h, 0, 0, 5);
	MoveLeg(fd, 2, 0, 0, h, 0, 0, 5);

	MoveLeg(fd, 0, 0, 0, h, 0, 0, 0);
	MoveLeg(fd, 4, 0, 0, h, 0, 0, 0);
	MoveLeg(fd, 2, 0, 0, h, 0, 0, 0);

	MoveLeg(fd, 0, 0, 0, h, 0, 0, -5);
	MoveLeg(fd, 4, 0, 0, h, 0, 0, -5);
	MoveLeg(fd, 2, 0, 0, h, 0, 0, -5);

	MoveLeg(fd, 0, 0, 0, h, 0, 0, -10);
	MoveLeg(fd, 4, 0, 0, h, 0, 0, -10);
	MoveLeg(fd, 2, 0, 0, h, 0, 0, -10);

	MoveLeg(fd, 0, 0, 0, h, 0, 0, -15);
	MoveLeg(fd, 4, 0, 0, h, 0, 0, -15);
	MoveLeg(fd, 2, 0, 0, h, 0, 0, -15);

	// down
	MoveLeg(fd, 5, 0, 0, h-20, 0, 0, 10);
	MoveLeg(fd, 1, 0, 0, h-20, 0, 0, 10);
	MoveLeg(fd, 3, 0, 0, h-20, 0, 0, 10);

	MoveLeg(fd, 5, 0, 0, h, 0, 0, 15);
	MoveLeg(fd, 1, 0, 0, h, 0, 0, 15);
	MoveLeg(fd, 3, 0, 0, h, 0, 0, 15);
}

void SwivelRight(int fd, int h)
{
	// lift
	MoveLeg(fd, 0, 0, 0, h-20, 0, 0, 15);
	MoveLeg(fd, 4, 0, 0, h-20, 0, 0, 15);
	MoveLeg(fd, 2, 0, 0, h-20, 0, 0, 15);

	MoveLeg(fd, 0, 0, 0, h-30, 0, 0, 10);
	MoveLeg(fd, 4, 0, 0, h-30, 0, 0, 10);
	MoveLeg(fd, 2, 0, 0, h-30, 0, 0, 10);

	MoveLeg(fd, 0, 0, 0, h-50, 0, 0, 5);
	MoveLeg(fd, 4, 0, 0, h-50, 0, 0, 5);
	MoveLeg(fd, 2, 0, 0, h-50, 0, 0, 5);

	MoveLeg(fd, 0, 0, 0, h-50, 0, 0, 0);
	MoveLeg(fd, 4, 0, 0, h-50, 0, 0, 0);
	MoveLeg(fd, 2, 0, 0, h-50, 0, 0, 0);

	MoveLeg(fd, 0, 0, 0, h-30, 0, 0, -5);
	MoveLeg(fd, 4, 0, 0, h-30, 0, 0, -5);
	MoveLeg(fd, 2, 0, 0, h-30, 0, 0, -5);

	// back
	MoveLeg(fd, 5, 0, 0, h, 0, 0, -10);
	MoveLeg(fd, 1, 0, 0, h, 0, 0, -10);
	MoveLeg(fd, 3, 0, 0, h, 0, 0, -10);

	MoveLeg(fd, 5, 0, 0, h, 0, 0, -5);
	MoveLeg(fd, 1, 0, 0, h, 0, 0, -5);
	MoveLeg(fd, 3, 0, 0, h, 0, 0, -5);

	MoveLeg(fd, 5, 0, 0, h, 0, 0, 0);
	MoveLeg(fd, 1, 0, 0, h, 0, 0, 0);
	MoveLeg(fd, 3, 0, 0, h, 0, 0, 0);

	MoveLeg(fd, 5, 0, 0, h, 0, 0, 5);
	MoveLeg(fd, 1, 0, 0, h, 0, 0, 5);
	MoveLeg(fd, 3, 0, 0, h, 0, 0, 5);

	MoveLeg(fd, 5, 0, 0, h, 0, 0, 10);
	MoveLeg(fd, 1, 0, 0, h, 0, 0, 10);
	MoveLeg(fd, 3, 0, 0, h, 0, 0, 10);

	MoveLeg(fd, 5, 0, 0, h, 0, 0, 15);
	MoveLeg(fd, 1, 0, 0, h, 0, 0, 15);
	MoveLeg(fd, 3, 0, 0, h, 0, 0, 15);

	// down
	MoveLeg(fd, 0, 0, 0, h-20, 0, 0, -10);
	MoveLeg(fd, 4, 0, 0, h-20, 0, 0, -10);
	MoveLeg(fd, 2, 0, 0, h-20, 0, 0, -10);

	MoveLeg(fd, 0, 0, 0, h, 0, 0, -15);
	MoveLeg(fd, 4, 0, 0, h, 0, 0, -15);
	MoveLeg(fd, 2, 0, 0, h, 0, 0, -15);

	// lift
	MoveLeg(fd, 5, 0, 0, h, 0, 0, 15);
	MoveLeg(fd, 1, 0, 0, h, 0, 0, 15);
	MoveLeg(fd, 3, 0, 0, h, 0, 0, 15);

	MoveLeg(fd, 5, 0, 0, h-20, 0, 0, 10);
	MoveLeg(fd, 1, 0, 0, h-20, 0, 0, 10);
	MoveLeg(fd, 3, 0, 0, h-20, 0, 0, 10);

	MoveLeg(fd, 5, 0, 0, h-30, 0, 0, 5);
	MoveLeg(fd, 1, 0, 0, h-30, 0, 0, 5);
	MoveLeg(fd, 3, 0, 0, h-30, 0, 0, 5);

	MoveLeg(fd, 5, 0, 0, h-50, 0, 0, 0);
	MoveLeg(fd, 1, 0, 0, h-50, 0, 0, 0);
	MoveLeg(fd, 3, 0, 0, h-50, 0, 0, 0);

	MoveLeg(fd, 5, 0, 0, h-30, 0, 0, -5);
	MoveLeg(fd, 1, 0, 0, h-30, 0, 0, -5);
	MoveLeg(fd, 3, 0, 0, h-30, 0, 0, -5);

	// back
	MoveLeg(fd, 0, 0, 0, h, 0, 0, -10);
	MoveLeg(fd, 4, 0, 0, h, 0, 0, -10);
	MoveLeg(fd, 2, 0, 0, h, 0, 0, -10);

	MoveLeg(fd, 0, 0, 0, h, 0, 0, -5);
	MoveLeg(fd, 4, 0, 0, h, 0, 0, -5);
	MoveLeg(fd, 2, 0, 0, h, 0, 0, -5);

	MoveLeg(fd, 0, 0, 0, h, 0, 0, 0);
	MoveLeg(fd, 4, 0, 0, h, 0, 0, 0);
	MoveLeg(fd, 2, 0, 0, h, 0, 0, 0);

	MoveLeg(fd, 0, 0, 0, h, 0, 0, 5);
	MoveLeg(fd, 4, 0, 0, h, 0, 0, 5);
	MoveLeg(fd, 2, 0, 0, h, 0, 0, 5);

	MoveLeg(fd, 0, 0, 0, h, 0, 0, 10);
	MoveLeg(fd, 4, 0, 0, h, 0, 0, 10);
	MoveLeg(fd, 2, 0, 0, h, 0, 0, 10);

	MoveLeg(fd, 0, 0, 0, h, 0, 0, 15);
	MoveLeg(fd, 4, 0, 0, h, 0, 0, 15);
	MoveLeg(fd, 2, 0, 0, h, 0, 0, 15);

	// down
	MoveLeg(fd, 5, 0, 0, h-20, 0, 0, -10);
	MoveLeg(fd, 1, 0, 0, h-20, 0, 0, -10);
	MoveLeg(fd, 3, 0, 0, h-20, 0, 0, -10);

	MoveLeg(fd, 5, 0, 0, h, 0, 0, -15);
	MoveLeg(fd, 1, 0, 0, h, 0, 0, -15);
	MoveLeg(fd, 3, 0, 0, h, 0, 0, -15);
}

void PitchForward(int fd, int h)
{
	MoveLeg(fd, 0, 0, 0, h, -10, 0, 0);
	MoveLeg(fd, 4, 0, 0, h, -10, 0, 0);
	MoveLeg(fd, 2, 0, 0, h, -10, 0, 0);

	MoveLeg(fd, 5, 0, 0, h, -10, 0, 0);
	MoveLeg(fd, 1, 0, 0, h, -10, 0, 0);
	MoveLeg(fd, 3, 0, 0, h, -10, 0, 0);
}

void PitchBackward(int fd, int h)
{
	MoveLeg(fd, 0, 0, 0, h, 10, 0, 0);
	MoveLeg(fd, 4, 0, 0, h, 10, 0, 0);
	MoveLeg(fd, 2, 0, 0, h, 10, 0, 0);

	MoveLeg(fd, 5, 0, 0, h, 10, 0, 0);
	MoveLeg(fd, 1, 0, 0, h, 10, 0, 0);
	MoveLeg(fd, 3, 0, 0, h, 10, 0, 0);
}

void RollRight(int fd, int h)
{
	MoveLeg(fd, 0, 0, 0, h, 0, -12, 0);
	MoveLeg(fd, 4, 0, 0, h, 0, -12, 0);
	MoveLeg(fd, 2, 0, 0, h, 0, -12, 0);

	MoveLeg(fd, 5, 0, 0, h, 0, -12, 0);
	MoveLeg(fd, 1, 0, 0, h, 0, -12, 0);
	MoveLeg(fd, 3, 0, 0, h, 0, -12, 0);
}

void RollLeft(int fd, int h)
{
	MoveLeg(fd, 0, 0, 0, h, 0, 12, 0);
	MoveLeg(fd, 4, 0, 0, h, 0, 12, 0);
	MoveLeg(fd, 2, 0, 0, h, 0, 12, 0);

	MoveLeg(fd, 5, 0, 0, h, 0, 12, 0);
	MoveLeg(fd, 1, 0, 0, h, 0, 12, 0);
	MoveLeg(fd, 3, 0, 0, h, 0, 12, 0);
}

void ShiftForward(int fd, int h)
{
	MoveLeg(fd, 0, 0, -40, h, 0, 0, 0);
	MoveLeg(fd, 5, 0, -40, h, 0, 0, 0);
	MoveLeg(fd, 4, 0, -40, h, 0, 0, 0);

	MoveLeg(fd, 1, 0, -40, h, 0, 0, 0);
	MoveLeg(fd, 2, 0, -40, h, 0, 0, 0);
	MoveLeg(fd, 3, 0, -40, h, 0, 0, 0);
}

void ShiftBackward(int fd, int h)
{
	MoveLeg(fd, 0, 0, 40, h, 0, 0, 0);
	MoveLeg(fd, 5, 0, 40, h, 0, 0, 0);
	MoveLeg(fd, 4, 0, 40, h, 0, 0, 0);

	MoveLeg(fd, 1, 0, 40, h, 0, 0, 0);
	MoveLeg(fd, 2, 0, 40, h, 0, 0, 0);
	MoveLeg(fd, 3, 0, 40, h, 0, 0, 0);
}

void ShiftRight(int fd, int h)
{
	MoveLeg(fd, 0, -40, 0, h, 0, 0, 0);
	MoveLeg(fd, 5, -40, 0, h, 0, 0, 0);
	MoveLeg(fd, 4, -40, 0, h, 0, 0, 0);

	MoveLeg(fd, 1, -40, 0, h, 0, 0, 0);
	MoveLeg(fd, 2, -40, 0, h, 0, 0, 0);
	MoveLeg(fd, 3, -40, 0, h, 0, 0, 0);
}

void ShiftLeft(int fd, int h)
{
	MoveLeg(fd, 0, 40, 0, h, 0, 0, 0);
	MoveLeg(fd, 5, 40, 0, h, 0, 0, 0);
	MoveLeg(fd, 4, 40, 0, h, 0, 0, 0);

	MoveLeg(fd, 1, 40, 0, h, 0, 0, 0);
	MoveLeg(fd, 2, 40, 0, h, 0, 0, 0);
	MoveLeg(fd, 3, 40, 0, h, 0, 0, 0);
}

void ShiftUp(int fd, int h)
{
	MoveLeg(fd, 0, 0, 0, h+40, 0, 0, 0);
	MoveLeg(fd, 5, 0, 0, h+40, 0, 0, 0);
	MoveLeg(fd, 4, 0, 0, h+40, 0, 0, 0);
	MoveLeg(fd, 1, 0, 0, h+40, 0, 0, 0);
	MoveLeg(fd, 2, 0, 0, h+40, 0, 0, 0);
	MoveLeg(fd, 3, 0, 0, h+40, 0, 0, 0);
}

void ShiftDown(int fd, int h)
{
	MoveLeg(fd, 0, 0, 0, h-40, 0, 0, 0);
	MoveLeg(fd, 5, 0, 0, h-40, 0, 0, 0);
	MoveLeg(fd, 4, 0, 0, h-40, 0, 0, 0);
	MoveLeg(fd, 1, 0, 0, h-40, 0, 0, 0);
	MoveLeg(fd, 2, 0, 0, h-40, 0, 0, 0);
	MoveLeg(fd, 3, 0, 0, h-40, 0, 0, 0);
}
void loop(int fd)
{
	while (radio.available())
	{
		radio.read(&data, sizeof(data));
		right_X_Val = data.right_X;
		right_Y_Val = data.right_Y;
		right_Button_Val = data.right_Button;
		left_X_Val = data.left_X;
		left_Y_Val = data.left_Y;
		left_Button_Val = data.left_Button;

		cam_Top_Val = map(data.left_Pot, 0, 255, 3456, 8256);
		cam_Base_Val = map(data.right_Pot, 255, 0, 2944, 10368);

		button_1_Val = data.button_1;
		button_2_Val = data.button_2;
		button_3_Val = data.button_3;

		maestroSetTarget(fd, 18, cam_Top_Val);
		maestroSetTarget(fd, 19, cam_Base_Val);

		if (button_1_Val == 1 && config_Mode == 1)
		{
			config_Mode = 2;
			cout << "MODE: " << config_Mode << endl;
		}
		else if (button_1_Val == 1 && config_Mode == 2)
		{
			config_Mode = 1;
			cout << "MODE: " << config_Mode << endl;
		}


//		 Turn on/off LED
		if (right_Button_Val == 0 && lastState == 1)
		{
			cout << "Right button: " << right_Button_Val << endl;
			lastState = 0;
			digitalWrite(LED, HIGH);
		}
		else if (right_Button_Val == 0 && lastState == 0)
		{
			cout << "Right button: " << right_Button_Val << endl;
			lastState = 1;
			digitalWrite(LED, LOW);
		}

		if (left_Button_Val == 0 && movement_Mode == 1)
		{
			cout << "Left button: " << left_Button_Val << endl;
			movement_Mode = 2;
			cout << "MODE: " << movement_Mode << endl;
		}
		else if (left_Button_Val == 0 && movement_Mode == 2)
		{
			cout << "Left button: " << left_Button_Val << endl;
			movement_Mode = 1;
			cout << "MODE: " << movement_Mode << endl;
		}

		switch (config_Mode)
		{
			case 1:
				if (button_2_Val == 1 && height > -90)
				{
					height-=10;
					cout << "Adjusting height: " << height << endl;
				}
				else if (button_3_Val == 1 && height < 50)
				{
					height+=10;
					cout << "Adjusting height: " << height << endl;
				}
				break;
			case 2:
				if (button_2_Val == 1 && speed > 10)
				{
					speed-=10;
					cout << "Adjusting speed: " << speed << endl;
				}
				else if (button_3_Val == 1 && speed < 100)
				{
					speed+=10;
					cout << "Adjusting speed: " << speed << endl;
				}
				for (int i = 0; i < 18; i++)
				{
					maestroSetSpeed(fd, i, speed);
				}
				break;
		}
		switch (movement_Mode)
		{
			case 1:
				if (right_Y_Val == 255)
				{
					cout << "Moving Forward: " << right_Y_Val << endl;
					MoveForward(fd, height);
				}
				else if (right_Y_Val == 0)
				{
					cout << "Moving Backward: " << right_Y_Val << endl;
					MoveBackward(fd, height);
				}
				else if (right_X_Val == 0)
				{
					cout << "Moving Right: " << right_X_Val << endl;
					MoveRight(fd, height);
				}
				else if (right_X_Val == 255)
				{
					cout << "Moving Left: " << right_X_Val << endl;
					MoveLeft(fd, height);
				}
				else if (left_Y_Val == 255)
				{
					cout << "Moving Forward: " << left_Y_Val << endl;
					ShiftUp(fd, height);
				}
				else if (left_Y_Val == 0)
				{
					cout << "Moving Backward: " << left_Y_Val << endl;
					ShiftDown(fd, height);
				}
				else if (left_X_Val == 0)
				{
					cout << "Swivel Right: " << left_X_Val << endl;
					SwivelRight(fd, height);
				}
				else if (left_X_Val == 255)
				{
					cout << "Swivel Left: " << left_X_Val << endl;
					SwivelLeft(fd, height);
				}
				else if (left_X_Val == 127 || left_Y_Val == 127 || right_X_Val == 127 || right_Y_Val == 127)
				{
					cout << "CENTER: " << right_X_Val << "  " << right_Y_Val << "  "
									   << left_X_Val << "  " << left_X_Val << endl;
					MoveLeg(fd, 0, 0, 0, height, 0, 0, 0);
					MoveLeg(fd, 5, 0, 0, height, 0, 0, 0);
					MoveLeg(fd, 4, 0, 0, height, 0, 0, 0);
					MoveLeg(fd, 1, 0, 0, height, 0, 0, 0);
					MoveLeg(fd, 2, 0, 0, height, 0, 0, 0);
					MoveLeg(fd, 3, 0, 0, height, 0, 0, 0);
				}
				break;
			case 2:
				if (right_Y_Val == 255)
				{
					cout << "Moving Forward: " << right_Y_Val << endl;
					ShiftForward(fd, height);
				}
				else if (right_Y_Val == 0)
				{
					cout << "Moving Backward: " << right_Y_Val << endl;
					ShiftBackward(fd, height);
				}
				else if (right_X_Val == 0)
				{
					cout << "Moving Right: " << right_X_Val << endl;
					ShiftRight(fd, height);
				}
				else if (right_X_Val == 255)
				{
					cout << "Moving Left: " << right_X_Val << endl;
					ShiftLeft(fd, height);
				}
				else if (left_Y_Val == 255)
				{
					cout << "Moving Forward: " << left_Y_Val << endl;
					PitchForward(fd, height);
				}
				else if (left_Y_Val == 0)
				{
					cout << "Moving Backward: " << left_Y_Val << endl;
					PitchBackward(fd, height);
				}
				else if (left_X_Val == 0)
				{
					cout << "Swivel Right: " << left_X_Val << endl;
					RollRight(fd, height);
				}
				else if (left_X_Val == 255)
				{
					cout << "Swivel Left: " << left_X_Val << endl;
					RollLeft(fd, height);
				}
				else if (left_X_Val == 127 || left_Y_Val == 127 || right_X_Val == 127 || right_Y_Val == 127)
				{
					cout << "CENTER: " << right_X_Val << "  " << right_Y_Val << "  "
									   << left_X_Val << "  " << left_X_Val << endl;
					MoveLeg(fd, 0, 0, 0, height, 0, 0, 0);
					MoveLeg(fd, 5, 0, 0, height, 0, 0, 0);
					MoveLeg(fd, 4, 0, 0, height, 0, 0, 0);
					MoveLeg(fd, 1, 0, 0, height, 0, 0, 0);
					MoveLeg(fd, 2, 0, 0, height, 0, 0, 0);
					MoveLeg(fd, 3, 0, 0, height, 0, 0, 0);
				}
				break;
		}
	}
}

int main(int argc, char** argv)
{
	setup();
	// Open the Maestro's virtual COM port
	const char* device = "/dev/ttyAMA0";

	int fd = open(device, O_RDWR | O_NOCTTY);
	if (fd == -1)
	{
		perror(device);
		return 1;
	}

	struct termios options;
	tcgetattr(fd, &options);

	options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
	options.c_oflag &= ~(ONLCR | OCRNL);
	options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tcsetattr(fd, TCSANOW, &options);

	for (int i = 0; i < 18; i++)
	{
		maestroSetSpeed(fd, i, 30);
	}

	maestroSetSpeed(fd, 18, 50);
	maestroSetSpeed(fd, 19, 50);
	wiringPiSetupGpio();
	pinMode(LED, OUTPUT);
	digitalWrite(LED, LOW);

	initHex(fd, 0, 0, -50, 0, 0, 0);

	while(1)
	{
		loop(fd);
	}
	return 0;
}
