#ifndef EXPLORE_MAP_H
#define EXPLORE_MAP_H

#include <stddef.h>

typedef enum _MapExplorationDir {
    X_POS,
    X_NEG,
    Y_POS,
    Y_NEG
} MapExplorationDir;

/**
 * Node define a tile in the Wave propagation exploration algorithm.
 * @param distance : represents the distance relative to neighbour tiles. This value
 * is modified by the wave propagation function and must not be altered with unless the 
 * destination has changed.
 * @param state : represents the state of a Node. State 0 : invalid (unexplored or obstructed)
 * State 1 : valid   (explored and free)
 */
typedef struct _Node {
    short int x = -1;
    short int y = -1;
    short int distance = -1;
    short int status = -1;
} Node;

/*Definition of dynamic array of Nodes*/
typedef struct _ArrayofNode {
  Node *array;
  size_t used;
  size_t size;

  void (*InitArray) (struct _ArrayofNode *a, size_t initialSize);
  void (*InsertArray) (struct _ArrayofNode *a, Node element);
  void (*FreeArray) (struct _ArrayofNode *a);

} ArrayofNode;
/***********************************/
extern void mInitArray(ArrayofNode *a, size_t initialSize);
extern void mInsertArray(ArrayofNode *a, Node element);
extern void mFreeArray(ArrayofNode *a); 

typedef struct _ExploreMap {
    int initX;
    int initY;
    int currX;
    int currY;

    Node mBase;
    Node mActiveNode;
    Node flowMap[50][50];
    ArrayofNode newNodes;
    

    uint8_t mapResolutionCM;
    uint8_t map[50][50];
    /* Member Functions */
    void (*Construct) (struct _ExploreMap *obj, int initX, int initY);
    void (*Move) (struct _ExploreMap *obj, int x, int y);
    int (*AddData) (struct _ExploreMap *obj, int y_neg, int x_pos, int y_pos, int x_neg);
    MapExplorationDir (*GetBestDir) (struct _ExploreMap *obj);
    void (*BuildFlow) (struct _ExploreMap *obj);
} ExploreMap;

/* Global Member Functions */
extern void mConstructor (ExploreMap *obj, int initX, int initY);
extern void mMove (ExploreMap *obj, int x, int y);
extern int mAddData (ExploreMap *obj, int y_neg, int x_pos, int y_pos, int x_neg);
extern MapExplorationDir mGetBestDir (ExploreMap *obj);
/**
 * This function runs the Wave Propagation algorithm starting at { baseX, baseY } tile.
 * WARNING/!\ - Calling this function will overwrite every distances in the current flowMap 
*/
extern void mBuildFlowMap(ExploreMap *obj);

/* Global Functions */
extern void ExploreMapNew(ExploreMap *obj);
extern void ArrayofNodeNew(ArrayofNode *a);

#endif