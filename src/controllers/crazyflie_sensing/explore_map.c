#include "explore_map.h"


/* New Function */
extern void ExploreMapNew(ExploreMap *obj) {
    obj->Construct = mConstructor;
    obj->Move      = mMove;
    obj->AddData   = mAddData;
    obj->GetBestDir= mGetBestDir;
    obj->BuildFlow = mBuildFlowMap;
    obj->NextNode  = mNextNode; 
}

/* Class Constructor */
extern void mConstructor (ExploreMap *obj, int initX, int initY) {
    obj->mapResolutionCM = MAX_ARENA_SIZE / MAP_SIZE; /* Number of CM per square */

    NodeArrayNew(&obj->newNodes);
    obj->newNodes.Init(&obj->newNodes, 16);

    NodeArrayNew(&obj->discovered);
    obj->discovered.Init(&obj->discovered, 16);

    /*For now base is (0,0)*/
    /*TODO: add base pos estimation*/
    obj->mBase = (Node){ 0, 0, 0, 1};

    obj->currX = (int) (initX * 100) / obj->mapResolutionCM;
    obj->currY = (int) (initY * 100) / obj->mapResolutionCM;

    obj->initX = obj->currX;
    obj->initY = obj->currY;
    
    for (unsigned int i = 0; i < MAP_SIZE; i++) {
        for (unsigned int j = 0; j < MAP_SIZE; j++) {
            obj->map[i][j] = 0;
            obj->distMap[i][j] = -1; //Must be < 0
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
    if (y_neg != -2) {
        obj->map[obj->currX][i] = 2;
    }

    /* Fill the map with x_pos */
    nEmpty = (x_pos != -2)? x_pos / obj->mapResolutionCM : 200 / obj->mapResolutionCM;
    for (i = obj->currX; i <= obj->currX + nEmpty && i < MAP_SIZE; i++){
        obj->map[i][obj->currY] = 1;  
    }
    if (x_pos != -2) {
        obj->map[i][obj->currY] = 2;
    }

    /* Fill the map with y_pos */
    nEmpty = (y_pos != -2)? y_pos / obj->mapResolutionCM : 200 / obj->mapResolutionCM;
    for (i = obj->currY; i <= obj->currY + nEmpty && i < MAP_SIZE; i++){
        obj->map[obj->currX][i] = 1;
    }
    if (y_pos != -2) {
        obj->map[obj->currX][i] = 2;
    }

    /* Fill the map with x_neg */
    nEmpty = (x_neg != -2)? x_neg / obj->mapResolutionCM : 200 / obj->mapResolutionCM;
    for (i = obj->currX; i >= obj->currX - nEmpty && i > 0; i--){
        obj->map[i][obj->currY] = 1;
    }
    if (x_neg != -2) {
        obj->map[i][obj->currY] = 2;
    }
    return 0;
}

extern MapExplorationDir mGetBestDir (ExploreMap *obj) {
    return MapExplorationDir::X_NEG;
}

extern void mBuildFlowMap(ExploreMap *obj) {
    /*Here we add the first Node corresponding to base position*/
    obj->newNodes.Insert(&obj->newNodes, obj->mBase);

    while(obj->mActiveNode.x != obj->currX && obj->mActiveNode.y != obj->currY && obj->newNodes.used != 0) {
        for(size_t i = 0; i < obj->newNodes.size; i++) {
            
            obj->mActiveNode = obj->newNodes.array[i];
            int x = obj->mActiveNode.x;
            int y = obj->mActiveNode.y;
            int dist = obj->mActiveNode.distance;
            
            /*Check for FRONT node*/
            if ((y - 1) > 0 && obj->distMap[x][y - 1] != -1) {
                obj->distMap[x][y - 1] = argos::Min(obj->distMap[x][y - 1], dist + 1);
                obj->discovered.Insert(&obj->discovered,(Node){x, y - 1, obj->distMap[x][y - 1], 1});
            }
            /*Check for LEFT node*/
            if ((x + 1) < MAP_SIZE && obj->distMap[x + 1][y] != -1) {
                obj->distMap[x + 1][y] = argos::Min(obj->distMap[x + 1][y], dist + 1);
                obj->discovered.Insert(&obj->discovered,(Node){x + 1, y, obj->distMap[x + 1][y], 1});
            }
            /*Check for BACK node*/
            if ((y + 1) < MAP_SIZE && obj->distMap[x][y + 1] != -1) {
                obj->distMap[x][y + 1] = argos::Min(obj->distMap[x][y + 1], dist + 1);
                obj->discovered.Insert(&obj->discovered,(Node){x, y + 1, obj->distMap[x][y + 1], 1});
            }
            /*Check for RIGHT node*/
            if ((x - 1) > 0 && obj->distMap[x - 1][y] != -1) {
                obj->distMap[x - 1][y] = argos::Min(obj->distMap[x - 1][y], dist + 1);
                obj->discovered.Insert(&obj->discovered,(Node){x - 1, y, obj->distMap[x - 1][y], 1});
            }
        }
        obj->newNodes.Free(&obj->newNodes);
        obj->newNodes = obj->discovered;
        obj->discovered.Free(&obj->discovered);
        obj->discovered.Init(&obj->discovered, 16);
    }
}

extern MapExplorationDir mNextNode(ExploreMap *obj) {
    int x = obj->currX;
    int y = obj->currY;
    int bestDist = 0;

    MapExplorationDir dir = NONE;

    /*Check for FRONT node*/
    if ((y - 1) > 0 && obj->distMap[x][y - 1] < bestDist) {
        bestDist = obj->distMap[x][y - 1];
        dir = Y_NEG; 
    }
    /*Check for LEFT node*/
    if ((x + 1) < MAP_SIZE && obj->distMap[x + 1][y] < bestDist) {
        bestDist = obj->distMap[x + 1][y];
        dir = X_POS; 
    }
    /*Check for BACK node*/
    if ((y + 1) < MAP_SIZE && obj->distMap[x][y + 1] < bestDist) {
        bestDist = obj->distMap[x][y + 1];
        dir = Y_POS; 
    }
    /*Check for RIGHT node*/
    if ((x - 1) > 0 && obj->distMap[x - 1][y] < bestDist) {
        bestDist = obj->distMap[x - 1][y];
        dir = X_NEG; 
    }
    return dir;
}
