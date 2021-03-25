#include "explore_map.h"

/* New Function */
extern void ExploreMapNew(ExploreMap *obj) {
    obj->Construct = mConstructor;
    obj->Move      = mMove;
    obj->AddData   = mAddData;
    obj->GetBestDir= mGetBestDir;
}

/* Class Constructor */
extern void mConstructor (ExploreMap *obj, int initX, int initY) {
    obj->mapResolutionCM = 20; /* Number of CM per square */
    obj->currX = (int) (initX * 100) / obj->mapResolutionCM;
    obj->currY = (int) (initY * 100) / obj->mapResolutionCM;
    for (unsigned int i = 0; i < 50; i++) {
        for (unsigned int j = 0; j < 50; j++) {
            obj->map[i][j] = 0;
        }
    }
}

/* Move the drone on the map */
extern void mMove (ExploreMap *obj, int x, int y) {
    obj->currX = (int) (initX * 100) / obj->mapResolutionCM;
    obj->currY = (int) (initY * 100) / obj->mapResolutionCM;
}

/* Add the data collected from the sensor */
extern int mAddData (ExploreMap *obj, int y_neg, int x_pos, int y_pos, int x_neg) {
    unsigned int nEmpty;
    int i;
    
    /* Fill the map with y_neg */
    nEmpty = (y_neg != -2)? y_neg / obj->mapResolutionCM : 200 / obj->mapResolutionCM;
    for (i = obj->currY; i >= obj->currY - nEmpty && i > 0; i--){
        obj->map[obj->currX][i] = 1;
    }
    if (y_neg != -2) {obj->map[obj->currX][i] = 2;}

    /* Fill the map with x_pos */
    nEmpty = (x_pos != -2)? x_pos / obj->mapResolutionCM : 200 / obj->mapResolutionCM;
    for (i = obj->currX; i <= obj->currX + nEmpty && i < 50; i++){
        obj->map[i][obj->currY] = 1;
    }
    if (x_pos != -2) {obj->map[i][obj->currY] = 2;}

    /* Fill the map with y_pos */
    nEmpty = (y_pos != -2)? y_pos / obj->mapResolutionCM : 200 / obj->mapResolutionCM;
    for (i = obj->currY; i <= obj->currY + nEmpty && i < 50; i++){
        obj->map[obj->currX][i] = 1;
    }
    if (y_pos != -2) {obj->map[obj->currX][i] = 2;}

    /* Fill the map with x_neg */
    nEmpty = (x_neg != -2)? x_neg / obj->mapResolutionCM : 200 / obj->mapResolutionCM;
    for (i = obj->currX; i >= obj->currX - nEmpty && i > 0; i--){
        obj->map[i][obj->currY] = 1;
    }
    if (x_neg != -2) {obj->map[i][obj->currY] = 2;}
    return 0;
}

extern MapExplorationDir mGetBestDir (ExploreMap *obj) {
    
    return MapExplorationDir::X_NEG;
}
