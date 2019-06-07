#include <iostream>
#include <math.h>
#include "InverseKinematics.h"
using namespace std;

LegAngles legAngle;
BodyPos body;

InverseKinematics::InverseKinematics(void)
{
//    cout << "--- InverseKinematics Test ---" << endl;
}

BodyPos InverseKinematics::BodyFK(int legNumb, int Pos_x, int Pos_y, float Rot_x, float Rot_y, float Rot_z)
{
	float Total_x, Total_y, DistBodyCenterFeet, Roll_z, Pitch_z, AngleBodyCenter_x;
	Total_x = INIT_FEET_POS[legNumb][0] + BODY_CENTER_OFFSETS[legNumb][0] + Pos_x;
	Total_y = INIT_FEET_POS[legNumb][1] + BODY_CENTER_OFFSETS[legNumb][1] + Pos_y;
	DistBodyCenterFeet = sqrt(pow(Total_x, 2) + pow(Total_y, 2));
	AngleBodyCenter_x = atan2(Total_y, Total_x);
	Roll_z = tan(Rot_y * M_PI / 180) * Total_x;
	Pitch_z = tan(Rot_x * M_PI / 180) * Total_y;

	body.BodyFK_x = cos(AngleBodyCenter_x + (Rot_z * M_PI / 180)) * DistBodyCenterFeet - Total_x;
	body.BodyFK_y = sin(AngleBodyCenter_x + (Rot_z * M_PI / 180)) * DistBodyCenterFeet - Total_y;
	body.BodyFK_z = Roll_z + Pitch_z;

	float test = cos(AngleBodyCenter_x + Rot_z * M_PI / 180) * DistBodyCenterFeet;
//	cout << "\n\n*******************\n"
//			"**     Leg " << legNumb+1 << "     **\n"
//			"*******************" << endl;
//	cout << "===== Body FK =====" << endl;
//	cout << "initFeetPos_x = " << INIT_FEET_POS[legNumb][0] << endl;
//	cout << "initFeetPos_y = " << INIT_FEET_POS[legNumb][1] << endl;
//	cout << "Total_x = " << Total_x << endl;
//	cout << "Total_y = " << Total_y << endl;
//	cout << "DistBodyCenterFeet = " << DistBodyCenterFeet << endl;
//	cout << "AngleBodyCenter = " << AngleBodyCenter_x << endl;
//	cout << "Roll_z = " << Roll_z << endl;
//	cout << "Pitch_z = " << Pitch_z << endl;
//	cout << "BodyFK_x = " << body.BodyFK_x << endl;
//	cout << "BodyFK_y = " << body.BodyFK_y << endl;
//	cout << "BodyFK_z = " << body.BodyFK_z << endl;
	return body;
}

LegAngles InverseKinematics::LegIK(int legNumb, int Pos_x, int Pos_y, int Pos_z, float Rot_x, float Rot_y, float Rot_z)
{

	BodyFK(legNumb, Pos_x, Pos_y, Rot_x, Rot_y, Rot_z);

	float temp_Coxa, temp_Femur, temp_Tibia;
	float newFeetPos_x, newFeetPos_y, newFeetPos_z;
	newFeetPos_x = INIT_FEET_POS[legNumb][0] + Pos_x + body.BodyFK_x;
	newFeetPos_y = INIT_FEET_POS[legNumb][1] + Pos_y + body.BodyFK_y;
	newFeetPos_z = INIT_FEET_POS[legNumb][2] + Pos_z + body.BodyFK_z;
	float CoxaDistFeet = sqrt(pow(newFeetPos_x, 2) + pow(newFeetPos_y, 2));

    float IKSW = getIKSW(CoxaDistFeet, newFeetPos_z);
    temp_Coxa = getCoxa(newFeetPos_y, newFeetPos_x);
    temp_Femur = getFemur(IKSW, newFeetPos_z, CoxaDistFeet);
    temp_Tibia = getTibia(IKSW);

    FinalAngles(legNumb, temp_Coxa, temp_Femur, temp_Tibia);
//    cout << "\n===== Leg IK =====" << endl;
//    cout << "newPos_x = " << newFeetPos_x << endl;
//    cout << "newPos_y = " << newFeetPos_y << endl;
//    cout << "newPos_z = " << newFeetPos_z << endl;
//    cout << "CoxaDistFeet = " << CoxaDistFeet << endl;
//    cout << "IKSW = " << IKSW << endl;
//    cout << "FinalCoxaAngle: " << legAngle.Coxa[legNumb] << endl;
//    cout << "FinalFemurAngle: " << legAngle.Femur[legNumb] << endl;
//    cout << "FinalTibiaAngle: " << legAngle.Tibia[legNumb] << endl;

    return legAngle;
}

float InverseKinematics::getCoxa(float newFeetPos_y, float newFeetPos_x)
{
    return toDegrees(atan2(newFeetPos_y, newFeetPos_x));
}

float InverseKinematics::getFemur(float IKSW, float newFeetPos_z, float CoxaDistFeet)
{
    float alpha_1 = atan2(CoxaDistFeet - LENGTH_COXA, newFeetPos_z);
    float alpha_2 = acos((pow(LENGTH_TIBIA, 2) - pow(LENGTH_FEMUR, 2) - pow(IKSW, 2)) / (-2 * LENGTH_FEMUR * IKSW));

    return toDegrees(alpha_1 + alpha_2);
}

float InverseKinematics::getTibia(float IKSW)
{
	return toDegrees(acos((pow(IKSW, 2) - pow(LENGTH_TIBIA, 2) - pow(LENGTH_FEMUR, 2)) / (-2 * LENGTH_TIBIA * LENGTH_FEMUR)));
}

float InverseKinematics::getIKSW(float CoxaDistFeet, float newFeetPos_z)
{
	return sqrt(pow(newFeetPos_z, 2) + pow(CoxaDistFeet - LENGTH_COXA, 2));
}

float InverseKinematics::toDegrees(float radian)
{
    return radian * 180 / M_PI;
}

LegAngles InverseKinematics::FinalAngles(int legNumb, float coxa, float femur, float tibia)
{
	switch (legNumb)
	{
		case 0:
			legAngle.Coxa[0] = coxa + 45;
			legAngle.Femur[0] = femur;
			legAngle.Tibia[0] = tibia;
			break;
		case 1:
			legAngle.Coxa[1] = coxa + 90;
			legAngle.Femur[1] = femur;
			legAngle.Tibia[1] = tibia;
			break;
		case 2:
			legAngle.Coxa[2] = coxa + 135;
			legAngle.Femur[2] = femur;
			legAngle.Tibia[2] = tibia;
			break;
		case 3:
			legAngle.Coxa[3] = coxa + 225;
			legAngle.Femur[3] = femur;
			legAngle.Tibia[3] = tibia;
			break;
		case 4:
			if (coxa > 0)
			{
				legAngle.Coxa[4] = coxa - 90;
			}
			else if (coxa < 0)
			{
				legAngle.Coxa[4] = coxa + 270;
			}
			legAngle.Femur[4] = femur;
			legAngle.Tibia[4] = tibia;
			break;
		case 5:
			legAngle.Coxa[5] = coxa - 45;
			legAngle.Femur[5] = femur;
			legAngle.Tibia[5] = tibia;
			break;
		default:
			cout << "Non-existing leg!!!" << endl;
	}
}
