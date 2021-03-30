#ifndef EXPLORE_MAP_H
#define EXPLORE_MAP_H

#include "node_array.h"
#include <argos3/core/utility/math/general.h>

#define MAP_SIZE 50
#define MAX_ARENA_SIZE 1000

/**
 * @brief 
 * @param X_POS : 0
 * @param X_NEG : 1
 * @param Y_POS : 2
 * @param Y_NEG : 1
 */
typedef enum _MapExplorationDir {
    X_POS,
    X_NEG,
    Y_POS,
    Y_NEG
} MapExplorationDir;

typedef struct _ExploreMap {
    int initX;
    int initY;
    int currX;
    int currY;

    Node mBase;
    Node mActiveNode;

    NodeArray newNodes;
    NodeArray discovered;

    uint8_t mapResolutionCM;

    uint8_t map[MAP_SIZE][MAP_SIZE];
    int distMap[MAP_SIZE][MAP_SIZE];

    /* Member Functions */
    void (*Construct) (struct _ExploreMap *obj, int initX, int initY);
    void (*Move) (struct _ExploreMap *obj, int x, int y);
    int (*AddData) (struct _ExploreMap *obj, int y_neg, int x_pos, int y_pos, int x_neg);
    MapExplorationDir (*GetBestDir) (struct _ExploreMap *obj);
    MapExplorationDir (*NextNode) (struct _ExploreMap *obj);
    void (*BuildFlow) (struct _ExploreMap *obj);
} ExploreMap;

/* Global Member Functions */
extern void mConstructor (ExploreMap *obj, int initX, int initY);
extern void mMove (ExploreMap *obj, int x, int y);
extern int mAddData (ExploreMap *obj, int y_neg, int x_pos, int y_pos, int x_neg);
extern MapExplorationDir mGetBestDir (ExploreMap *obj);
extern MapExplorationDir mNextNode(ExploreMap *obj);
/**
 * @brief This function runs the Wave Propagation algorithm starting at { baseX, baseY } tile.
 * WARNING/!\ - Calling this function will overwrite every distances in the current distMap.
 * @param obj : reference to self. 
 */
extern void mBuildFlowMap(ExploreMap *obj);

/* Global Functions */
/**
 * @brief Construct a ExploreMap type object. This needs to be called in order to use internal functions
 * for a desired object.
 * 
 * @param obj referece to self.
 */
extern void ExploreMapNew(ExploreMap *obj);

#endif