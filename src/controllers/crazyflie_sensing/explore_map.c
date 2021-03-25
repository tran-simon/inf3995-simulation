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
    obj->currX = (int) initX / obj->mapResolutionCM;
    obj->currY = (int) initY / obj->mapResolutionCM;
    obj->initX = obj->currX;
    obj->initY = obj->currY;
    for (unsigned int i = 0; i < 50; i++) {
        for (unsigned int j = 0; j < 50; j++) {
            obj->map[i][j] = 0;
        }
    }
}

/* Move the drone on the map */
extern void mMove (ExploreMap *obj, int x, int y) {
    obj->currX = (int) x / obj->mapResolutionCM;
    obj->currY = (int) y / obj->mapResolutionCM;
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
    for (i = obj->currX; i <= obj->currX + nEmpty && i < 49; i++){
        obj->map[i][obj->currY] = 1;
    }
    if (x_pos != -2) {obj->map[i][obj->currY] = 2;}

    /* Fill the map with y_pos */
    nEmpty = (y_pos != -2)? y_pos / obj->mapResolutionCM : 200 / obj->mapResolutionCM;
    for (i = obj->currY; i <= obj->currY + nEmpty && i < 49; i++){
        obj->map[obj->currX][i] = 1;
    }
    if (y_pos != -2) {obj->map[obj->currX][i] = 2;}

    /* Fill the map with x_neg */
    nEmpty = (x_neg != -2)? x_neg / obj->mapResolutionCM : 200 / obj->mapResolutionCM;
    for (i = obj->currX; i >= obj->currX - nEmpty && i > 0; i--){
        obj->map[i][obj->currY] = 1;
    }
    if (x_neg != -2) {obj->map[i][obj->currY] = 2;}
    return nEmpty;
}

int max(int a, int b) {
    return (a >= b)? a: b;
}

extern MapExplorationDir mGetBestDir (ExploreMap *obj) {
    /***
    *   We want to go in the dir with the biggest potential information gain
    **/

    /* Check if y_neg is a good direction */
    int y_neg_sum = 0;
    int boxValue = -1;
    unsigned int i = obj->currX;
    unsigned int j = obj->currY;
    while (boxValue != 2) {
        if (j == 0) {boxValue = 2; break;}

        boxValue = obj->map[i][j];
        if (boxValue == 0) {y_neg_sum += 4;}
        if (boxValue == 1) {y_neg_sum += 1;}
        
        j--;
    }

    /* Check if x_pos is a good direction */
    int x_pos_sum = 0;
    boxValue = -1;
    i = obj->currX;
    j = obj->currY;
    while (boxValue != 2) {
        if (i == 49) {boxValue = 2; break;}

        boxValue = obj->map[i][j];
        if (boxValue == 0) {x_pos_sum += 4;}
        if (boxValue == 1) {x_pos_sum += 1;}
        
        i++;
    }

    /* Check if y_pos is a good direction */
    int y_pos_sum = 0;
    boxValue = -1;
    i = obj->currX;
    j = obj->currY;
    while (boxValue != 2) {
        if (j == 49) {boxValue = 2; break;}

        boxValue = obj->map[i][j];
        if (boxValue == 0) {y_pos_sum += 4;}
        if (boxValue == 1) {y_pos_sum += 1;}
        
        j++;
    }

    /* Check if x_neg is a good direction */
    int x_neg_sum = 0;
    boxValue = -1;
    i = obj->currX;
    j = obj->currY;
    while (boxValue != 2) {
        if (i == 0) {boxValue = 2; break;}

        boxValue = obj->map[i][j];
        if (boxValue == 0) {x_neg_sum += 4;}
        if (boxValue == 1) {x_neg_sum += 1;}
        
        i--;
    }

    int maxSum = max(y_neg_sum, max(x_pos_sum, max(y_pos_sum, x_neg_sum)));
    if (y_neg_sum == maxSum) { return MapExplorationDir::Y_NEG; }
    if (x_pos_sum == maxSum) { return MapExplorationDir::X_POS; }
    if (y_pos_sum == maxSum) { return MapExplorationDir::Y_POS; }
    if (x_neg_sum == maxSum) { return MapExplorationDir::X_NEG; }
}
