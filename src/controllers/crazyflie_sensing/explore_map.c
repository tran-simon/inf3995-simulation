#include "explore_map.h"

/* New Function */
extern void ExploreMapNew(ExploreMap *obj) {
    obj->Construct = mConstructor;
    obj->Move      = mMove;
    obj->AddData   = mAddData;
    obj->GetBestDir= mGetBestDir;
    obj->BuildFlow = mBuildFlowMap;
}

extern void ArrayofNodeNew(ArrayofNode *a) {
    a->InitArray = mInitArray;
    a->InsertArray = mInsertArray;
    a->FreeArray = mFreeArray;
}

/* Class Constructor */
extern void mConstructor (ExploreMap *obj, int initX, int initY) {
    obj->mapResolutionCM = 20; /* Number of CM per square */
    obj->newNodes.InitArray(&obj->newNodes, 16);

    /*For now base is (0,0)*/
    /*TODO: add base pos estimation*/
    obj->mBase = {
        x: 0,
        y: 0,
        distance: 1,
        status: 1
    };

    obj->currX = (int) (initX * 100) / obj->mapResolutionCM;
    obj->currY = (int) (initY * 100) / obj->mapResolutionCM;

    obj->initX = obj->currX;
    obj->initY = obj->currY;

    for (unsigned int i = 0; i < 50; i++) {
        for (unsigned int j = 0; j < 50; j++) {
            obj->map[i][j] = 0;
            obj->flowMap[i][j].x = i;
            obj->flowMap[i][j].y = j;
            obj->flowMap[i][j].distance = 0;
            obj->flowMap[i][j].status = 0;
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
        obj->flowMap[obj->currX][i].status = 1;
    }
    if (y_neg != -2) {
        obj->map[obj->currX][i] = 2;
        obj->flowMap[obj->currX][i].status = 0;
    }

    /* Fill the map with x_pos */
    nEmpty = (x_pos != -2)? x_pos / obj->mapResolutionCM : 200 / obj->mapResolutionCM;
    for (i = obj->currX; i <= obj->currX + nEmpty && i < 50; i++){
        obj->map[i][obj->currY] = 1;
        obj->flowMap[i][obj->currY].status = 1;
        
    }
    if (x_pos != -2) {
        obj->map[i][obj->currY] = 2;
        obj->flowMap[i][obj->currY].status = 0;
    }

    /* Fill the map with y_pos */
    nEmpty = (y_pos != -2)? y_pos / obj->mapResolutionCM : 200 / obj->mapResolutionCM;
    for (i = obj->currY; i <= obj->currY + nEmpty && i < 50; i++){
        obj->map[obj->currX][i] = 1;
        obj->flowMap[obj->currX][i].status = 1;
    }
    if (y_pos != -2) {
        obj->map[obj->currX][i] = 2;
        obj->flowMap[obj->currX][i].status = 0;
    }

    /* Fill the map with x_neg */
    nEmpty = (x_neg != -2)? x_neg / obj->mapResolutionCM : 200 / obj->mapResolutionCM;
    for (i = obj->currX; i >= obj->currX - nEmpty && i > 0; i--){
        obj->map[i][obj->currY] = 1;
        obj->flowMap[i][obj->currY].status= 1;
    }
    if (x_neg != -2) {
        obj->map[i][obj->currY] = 2;
        obj->flowMap[i][obj->currY].status = 0;
    }
    return 0;
}

extern MapExplorationDir mGetBestDir (ExploreMap *obj) {
    return MapExplorationDir::X_NEG;
}

extern void mBuildFlowMap(ExploreMap *obj) {

    /*Here we add the first Node corresponding to base position*/
    ArrayofNode tmpDiscoveries;
    tmpDiscoveries.InitArray(&tmpDiscoveries, 16);

    while(obj->mActiveNode.x != obj->currX && obj->mActiveNode.y != obj->currY && obj->mActiveNode.status != -1) {
        for(size_t i = 0; i < obj->newNodes.size; i++) {
            
            obj->mActiveNode = obj->newNodes[i];
            
            /*Check for FRONT node*/
            if(((obj->mActiveNode.y - 1) > 0) && (obj->flowMap[obj->mActiveNode.x][obj->mActiveNode.y - 1].status == 1) 
                                              && (obj->flowMap[obj->mActiveNode.x][obj->mActiveNode.y - 1].distance == 0)) {
                //obj->flowMap[obj->mActiveNode.x][obj->mActiveNode.y - 1].distance = obj->mActiveNode.distance + 1;
            }

            /*Check for LEFT node*/
            if(((obj->mActiveNode.x + 1) <= 49) && (obj->flowMap[obj->mActiveNode.x + 1][obj->mActiveNode.y].status == 1)
                                                && (obj->flowMap[obj->mActiveNode.x + 1][obj->mActiveNode.y].distance == 0)) {
                //obj->flowMap[obj->mActiveNode.x + 1][obj->mActiveNode.y].distance = obj->mActiveNode.distance + 1;
            } 

            /*Check for BACK node*/
            if(((obj->mActiveNode.y + 1) <= 49) && (obj->flowMap[obj->mActiveNode.x][obj->mActiveNode.y + 1].status == 1)
                                                && (obj->flowMap[obj->mActiveNode.x][obj->mActiveNode.y + 1].distance == 0)) {
                //obj->flowMap[obj->mActiveNode.x][obj->mActiveNode.y + 1].distance = obj->mActiveNode.distance + 1;
            }

            /*Check for left node*/
            if(((obj->mActiveNode.x - 1) > 0) && (obj->flowMap[obj->mActiveNode.x - 1][obj->mActiveNode.y].status == 1)
                                              && (obj->flowMap[obj->mActiveNode.x - 1][obj->mActiveNode.y].distance == 0)) {
                tmpDiscoveries.InsertArray(&tmpDiscoveries, obj->mActiveNode);
                //obj->flowMap[obj->mActiveNode.x - 1][obj->mActiveNode.y].distance = obj->mActiveNode.distance + 1;
            }
        }
    }
}

extern void mInitArray(ArrayofNode *a, size_t initialSize) {
    a->array = (Node *)malloc(initialSize * sizeof(Node));
    a->used = 0;
    a->size = initialSize;
}

extern void mInsertArray(ArrayofNode *a, Node element) {
    char cmpChar = '0';
    /*This is ungodly. We should implement linked list or some kind of iterable container to make an
      optimized sort algorithm and a remove functionality.
    */
    if(a->used != 0) {
        for(int i = 0; i < a->used; i++) {
            if(a->array[i].x == element.x && a->array[i].y  == element.y && a->array[i].distance == element.distance) {
                cmpChar = '1';
                break;
            }
        }
    }
    if (a->used == a->size) {
        a->size *= 2;
        a->array = (Node *)realloc(a->array, a->size * sizeof(Node));
    }
    if(cmpChar == '0') {
        a->array[a->used++] = element;
    }       
}

extern void mFreeArray(ArrayofNode *a) {
    free(a->array);
    a->array = NULL;
    a->used = a->size = 0;
}
