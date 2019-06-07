#ifndef INVERSE_KINEMATICS_H_
#define INVERSE_KINEMATICS_H_
#include <math.h>

// Leg Dimensions
const float LENGTH_COXA		=	40;
const float LENGTH_FEMUR	=	80;
const float LENGTH_TIBIA	=	165;

// Cosine
const float COS = cos(45 * M_PI / 180) * (LENGTH_COXA + LENGTH_FEMUR);

// Sine
const float SIN = sin(45 * M_PI / 180) * (LENGTH_COXA + LENGTH_FEMUR);

// Body Center Offsets						   x     y
const float BODY_CENTER_OFFSETS[6][2]	=	{ 70,  118,
											  82,   0,
											  70, -118,
											 -70, -118,
											 -82,   0,
											 -70,  118 };

// Initial Feet Positions					x     y    z
const float INIT_FEET_POS[6][3]		=	{ COS,  SIN, 165,
										  120,   0,  165,
										  COS, -SIN, 165,
										 -COS, -SIN, 165,
										 -120,   0,  165,
										 -COS,  SIN, 165 };

// Inputs
struct Inputs
{
	int Pos_x;
	int Pos_y;
	int Pos_z;
	float Rot_x;
	float Rot_y;
	float Rot_z;
};

struct LegAngles
{
	float Coxa[6];
	float Femur[6];
	float Tibia[6];
};

struct BodyPos
{
	int BodyFK_x;
	int BodyFK_y;
	int BodyFK_z;
};
class InverseKinematics
{
public:
	InverseKinematics();
	LegAngles LegIK(int legNumb, int Pos_x, int Pos_y, int Pos_z, float Rot_x, float Rot_y, float Rot_z);
	BodyPos BodyFK(int legNumb, int Pos_x, int Pos_y, float Rot_x, float Rot_y, float Rot_z);
	LegAngles FinalAngles(int legNumb, float coxa, float femur, float tibia);
private:
	float toDegrees(float radius);
	float toRadians(float degrees);
	float getCoxa(float newFeetPos_y, float newFeetPos_x);
	float getFemur(float IKSW, float newFeetPos_z, float CoxaDistFeet);
	float getTibia(float IKSW);
	float getIKSW(float CoxaDistFeet, float newFeetPos_z);
};
#endif
