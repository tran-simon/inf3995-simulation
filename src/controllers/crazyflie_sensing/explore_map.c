#include "explore_map.h"

/* New Function */
extern void ExploreMapNew(ExploreMap *obj) {
    obj->Construct = mConstructor;
    obj->Move      = mMove;
    obj->AddData   = mAddData;
}

/* Class Constructor */
extern void mConstructor (ExploreMap *obj, int initX, int initY) {
    obj->mapResolutionCM = 20; /* Number of CM per square */
    obj->currX = (initX * 100) / obj->mapResolutionCM;
    obj->currY = (initY * 100) / obj->mapResolutionCM;
    for (unsigned int i = 0; i < 30; i++) {
        for (unsigned int j = 0; j < 30; j++) {
            obj->map[i][j] = 0;
        }
    }
}

/* Move the drone on the map */
extern void mMove (ExploreMap *obj, int x, int y) {
    obj->currX = obj->currX + ((x * 100) / obj->mapResolutionCM);
    obj->currY = obj->currY + ((y * 100) / obj->mapResolutionCM);
}

/* Add the data collected from the sensor */
extern void mAddData (ExploreMap *obj, int frontDist, int leftDist, int backDist, int rightDist) {
    unsigned int nEmpty;
    int i;

    /* Fill the map with frontDist */
    nEmpty = (frontDist != -2)? frontDist / obj->mapResolutionCM : 200 / obj->mapResolutionCM;
    for (i = obj->currY; i >= obj->currY - nEmpty; i--){
        obj->map[obj->currX][i] = 1;
    }
    if (frontDist != -2) {obj->map[i][obj->currY] = 2;}

    /* Fill the map with leftDist */
    nEmpty = (leftDist != -2)? leftDist / obj->mapResolutionCM : 200 / obj->mapResolutionCM;
    for (i = obj->currX; i <= obj->currX + nEmpty; i++){
        obj->map[i][obj->currY] = 1;
    }
    if (leftDist != -2) {obj->map[i][obj->currY] = 2;}

    /* Fill the map with backDist */
    nEmpty = (backDist != -2)? backDist / obj->mapResolutionCM : 200 / obj->mapResolutionCM;
    for (i = obj->currY; i <= obj->currX + nEmpty; i++){
        obj->map[obj->currX][i] = 1;
    }
    if (backDist != -2) {obj->map[i][obj->currY] = 2;}

    /* Fill the map with rightDist */
    nEmpty = (rightDist != -2)? rightDist / obj->mapResolutionCM : 200 / obj->mapResolutionCM;
    for (i = obj->currX; i <= obj->currX + nEmpty; i++){
        obj->map[i][obj->currY] = 1;
    }
    if (rightDist != -2) {obj->map[i][obj->currY] = 2;}
}

