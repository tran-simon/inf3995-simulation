#include "crazyflie_qt_user_functions.h"

using namespace argos;

/****************************************/
/****************************************/

void CCrazyflieQTUserFunctions::DrawInWorld() {
    enum Orientation { horizontal, vertical };
    CRandom::CRNG* rng = CRandom::CreateRNG("argos");
    Orientation orientation = static_cast<Orientation>(rng->Uniform(CRange<UInt32>(0, 2)));

    for (SInt32 i = -1; i <= 1; i+=2) {
        UInt32 nWall = rng->Uniform(CRange<UInt32>(1, 3));
        Real allPos[nWall + 2]; allPos[0] = -5.0f; allPos[nWall + 1] = 5.0f;

        //Walls
        for (UInt32 j = 0; j < nWall; j++) {
            Real pos = rng->Gaussian(0.5, ((10.0 / nWall) * (j + 0.5)) - 5.0);
            allPos[j + 1] = pos;

            // Set the coord for horizontal/vertical arena
            CVector3 c_position;
            CVector3 c_size;
            if (orientation == Orientation::horizontal) {
                c_position = CVector3(i * 3.0f, pos, 0.0f);
                c_size = CVector3(4.0f, 0.1f, 2.0f);
            } else {
                c_position = CVector3(pos, i * 3.0f, 0.0f);
                c_size = CVector3(0.1f, 4.0f, 2.0f);
            }

            // Draw the wall(s)
            DrawBox(
                c_position, 
                CQuaternion(),
                c_size,
                CColor::BLACK);
        }

        //Doors
        for (UInt32 j = 0; j < nWall + 1; j++) {
            Real doorWidth = 0.6;
            Real doorPos = (allPos[j + 1] - allPos[j]) / 4;
            Real wallSize = (allPos[j + 1] - allPos[j]) / 2 - doorWidth;

            // Set the coord for horizontal/vertical arena
            CVector3 c_position1;
            CVector3 c_position2;
            CVector3 c_size;
            if (orientation == Orientation::horizontal) {
                c_position1 = CVector3(i * 1.0f, allPos[j] + doorPos - (doorWidth / 2), 0.0f);
                c_position2 = CVector3(i * 1.0f, allPos[j+1] - doorPos + (doorWidth / 2), 0.0f);
                c_size = CVector3(0.1f, wallSize, 2.0f);
            } else {
                c_position1 = CVector3(allPos[j] + doorPos - (doorWidth / 2), i * 1.0f, 0.0f);
                c_position2 = CVector3(allPos[j+1] - doorPos + (doorWidth / 2), i * 1.0f, 0.0f);
                c_size = CVector3(wallSize, 0.1f, 2.0f);
            }

            // Draw the walls to form the doors
            DrawBox(
                c_position1, 
                CQuaternion(),
                c_size,
                CColor::BLACK);
            DrawBox(
                c_position2, 
                CQuaternion(),
                c_size,
                CColor::BLACK);
        } 
    }    
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CCrazyflieQTUserFunctions, "crazyflie_qt_user_functions")