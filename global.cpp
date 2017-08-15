#include "global.h"


char * camera_name = "sr300";
String file_name = nullptr;
std::ofstream os("..\\OpenARK_test\\fingertips_openark.txt");

/***
 *  Intrinsics for Creative Senz3D camera
 *  for CVAR egocentric dataset
 */
extern double FX = 224.501999;
extern double FY = 230.494003;
extern double CX = 160.000000;
extern double CY = 120.000000;

//F200 camera multi-user ego-centric dataset
/*const auto FX = 478.01f;
const auto FY = 478.01f;
const auto CX = 321.59f;
const auto CY = 250.55f;*/
