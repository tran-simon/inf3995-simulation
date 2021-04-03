#include "explore_map.h"
#include <argos3/core/utility/logging/argos_log.h>

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
    obj->testValue = 1;
    
    obj->newNodes = (NodeArray *)malloc(sizeof(NodeArray));
    NodeArrayNew(obj->newNodes);
    obj->newNodes->Init(obj->newNodes, 16);

    obj->discovered =(NodeArray *)malloc(sizeof(NodeArray));
    NodeArrayNew(obj->discovered);
    obj->discovered->Init(obj->discovered, 16);

    /*For now base is (0,0)*/
    /*TODO: add base pos estimation*/
    obj->mBase = {49,49,1,1};

    obj->currX = (int) (initX * 100) / obj->mapResolutionCM;
    obj->currY = (int) (initY * 100) / obj->mapResolutionCM;

    obj->initX = obj->currX;
    obj->initY = obj->currY;
    
    for (unsigned int i = 0; i < MAP_SIZE; i++) {
        for (unsigned int j = 0; j < MAP_SIZE; j++) {
            //obj->map[i][j] = 0;
            obj->distMap[i][j] = (obj->map[i][j] == 0) ? -1:0;
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
        obj->distMap[obj->currX][i] = 0;
    }
    if (y_neg != -2) {
        obj->map[obj->currX][i] = 2;
    }

    /* Fill the map with x_pos */
    nEmpty = (x_pos != -2)? x_pos / obj->mapResolutionCM : 200 / obj->mapResolutionCM;
    for (i = obj->currX; i <= obj->currX + nEmpty && i < MAP_SIZE; i++){
        obj->map[i][obj->currY] = 1;  
        obj->distMap[i][obj->currY] = 0;  
    }
    if (x_pos != -2) {
        obj->map[i][obj->currY] = 2;
    }

    /* Fill the map with y_pos */
    nEmpty = (y_pos != -2)? y_pos / obj->mapResolutionCM : 200 / obj->mapResolutionCM;
    for (i = obj->currY; i <= obj->currY + nEmpty && i < MAP_SIZE; i++){
        obj->map[obj->currX][i] = 1;
        obj->distMap[obj->currX][i] = 0;
    }
    if (y_pos != -2) {
        obj->map[obj->currX][i] = 2;
    }

    /* Fill the map with x_neg */
    nEmpty = (x_neg != -2)? x_neg / obj->mapResolutionCM : 200 / obj->mapResolutionCM;
    for (i = obj->currX; i >= obj->currX - nEmpty && i > 0; i--){
        obj->map[i][obj->currY] = 1;
        obj->distMap[i][obj->currY] = 0;
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
    // Here we add the first Node corresponding to base position
    obj->newNodes->Insert(obj->newNodes, obj->mBase);
    obj->distMap[obj->mBase.x][obj->mBase.y] = obj->mBase.distance;

    // We verify that the drone isn't out of the map
    if(obj->currX > -1 && obj->currX < MAP_SIZE && obj->currY > -1 && obj->currY < MAP_SIZE) {
        // We run the Wave Propagation algorithm until a clear path is found between the drone
        // and the current base.

        char stop = 'n';
        while(stop == 'n') {
            int newNodeCount = 0;
            for (size_t i = 0; i < obj->newNodes->used; i++) {

                int x = obj->newNodes->array[i].x;
                int y = obj->newNodes->array[i].y;
                int dist = obj->newNodes->array[i].distance;

                // We check if the last added node was the drone position
                if (x == obj->currX && y == obj->currY) {
                    stop = 'y';
                    break;
                } 

                // For every direction we check if the current node is in the map,
                // if the current node isn't a wall and if it isn't the base

                // Check for FRONT node
                if ((y - 1) > -1 && obj->distMap[x][y - 1] > -1 && obj->distMap[x][y - 1] != 1) {
                    if(obj->distMap[x][y - 1] == 0) {
                        // A new node with a 0 distance has been discovered
                        obj->distMap[x][y - 1] = dist + 1;
                        newNodeCount++;
                    } else {
                        obj->distMap[x][y - 1] = argos::Min(obj->distMap[x][y - 1], dist + 1);
                    }
                    // We add the newly discovered node to the discovered array
                    obj->discovered->Insert(obj->discovered, {x, (y - 1), obj->distMap[x][y - 1], 1});
                }

                // Check for LEFT node
                if ((x + 1) < MAP_SIZE && obj->distMap[x + 1][y] > -1 && obj->distMap[x + 1][y] != 1) {
                    if(obj->distMap[x + 1][y] == 0) {
                        // A new node with a 0 distance has been discovered
                        obj->distMap[x + 1][y] = dist + 1;
                        newNodeCount++;
                    } else {
                        obj->distMap[x + 1][y] = argos::Min(obj->distMap[x + 1][y], dist + 1);
                    }
                    // We add the newly discovered node to the discovered array
                    obj->discovered->Insert(obj->discovered, {(x + 1), y, obj->distMap[x + 1][y], 1});
                }

                // Check for BACK node
                if ((y + 1) < MAP_SIZE && obj->distMap[x][y + 1] > -1 && obj->distMap[x][y + 1] != 1) {
                    if(obj->distMap[x][y + 1] == 0) {
                        // A new node with a 0 distance has been discovered
                        obj->distMap[x][y + 1] = dist + 1;
                        newNodeCount++;
                    } else {
                        obj->distMap[x][y + 1] = argos::Min(obj->distMap[x][y + 1], dist + 1);
                    }
                    // We add the newly discovered node to the discovered array
                    obj->discovered->Insert(obj->discovered, {x, (y + 1), obj->distMap[x][y + 1], 1});
                }

                // Check for RIGHT node
                if ((x - 1) > -1 && obj->distMap[x - 1][y] > -1 && obj->distMap[x - 1][y] != 1) {
                    if(obj->distMap[x - 1][y] == 0) {
                        // A new node with a 0 distance has been discovered
                        obj->distMap[x - 1][y] = dist + 1;
                        newNodeCount++;
                    } else {
                        obj->distMap[x - 1][y] = argos::Min(obj->distMap[x - 1][y], dist + 1);
                    }
                    // We add the newly discovered node to the discovered array
                    obj->discovered->Insert(obj->discovered, {(x - 1), y, obj->distMap[x - 1][y], 1});
                }
            }
            // Check to see if no newly discovered nodes have been found. Normally at least one node must
            // have a distance value of 0 for the algorithm to continue.
            if(newNodeCount == 0) {
                argos::LOG << "No new node has been discovered. Search closed." << std::endl;
                stop = 'y';
            }
            if(stop == 'n') {
                // We empty the current newNodes array and add the newly discovered ones to it.
                obj->newNodes->Free(obj->newNodes);
                obj->newNodes->Init(obj->newNodes, 16);

                for (int i = 0; i < obj->discovered->used; i++){
                    obj->newNodes->Insert(obj->newNodes, obj->discovered->array[i]);
                }

                obj->discovered->Free(obj->discovered);
                obj->discovered->Init(obj->discovered, 16);
            }
        }
    }
    // We free the allocated memory
    obj->newNodes->Free(obj->newNodes);
    obj->discovered->Free(obj->discovered);
}

extern MapExplorationDir mNextNode(ExploreMap *obj) {
    // Local variables
    MapExplorationDir dir = NONE;
    int x = obj->currX;
    int y = obj->currY;
    int bestDist = (MAP_SIZE * MAP_SIZE) + 1;

    // We make sure we are not already at destination
    if (!(x == obj->mBase.x && y == obj->mBase.y)) {
        // Check for FRONT node
        if ((y - 1) > -1 && obj->distMap[x][y - 1] < bestDist) {
            bestDist = obj->distMap[x][y - 1];
            dir = Y_NEG; 
        }

        // Check for LEFT node
        if ((x + 1) < MAP_SIZE && obj->distMap[x + 1][y] < bestDist) {
            bestDist = obj->distMap[x + 1][y];
            dir = X_POS; 
        }

        // Check for BACK node
        if ((y + 1) < MAP_SIZE && obj->distMap[x][y + 1] < bestDist) {
            bestDist = obj->distMap[x][y + 1];
            dir = Y_POS; 
        }

        // Check for RIGHT node
        if ((x - 1) > -1 && obj->distMap[x - 1][y] < bestDist) {
            bestDist = obj->distMap[x - 1][y];
            dir = X_NEG; 
        }
    }
    return dir;
}
