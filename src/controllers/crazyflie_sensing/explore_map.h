#ifndef EXPLORE_MAP_H
#define EXPLORE_MAP_H

#include "node_array.h"
#include "node.h"
#include <argos3/core/utility/math/general.h>

#define MAP_SIZE 50
#define MAX_ARENA_SIZE 1000

/**
 * @brief Map exploration directions
 * @param X_POS : 0
 * @param X_NEG : 1
 * @param Y_POS : 2
 * @param Y_NEG : 3
**/
typedef enum _MapExplorationDir {
   X_POS,
   X_NEG,
   Y_POS,
   Y_NEG, 
   NONE
} MapExplorationDir;

typedef struct _ExploreMap {
   /* Represents the initial x and y position of the drone in the map.
      These values shouldn't be modified.*/
   int initX, initY;

   /* Represents the current x and y position of the drone in the map. 
      These values are updated by calling Move(). */
   int currX, currY;

   /* Map resolution in cm/square */
   uint8_t mapResolutionCM;

   /* The current 2D exploration map. */
   uint8_t map[MAP_SIZE][MAP_SIZE];

   /* Node that represents the base in the distances array */
   struct Node mBase = {0,0,0,0};

   /* List of all the nodes discovered on the last step of the algorithm (should not contain duplicates) */
   NodeArray* newNodes;

   /* List of newly discovered nodes that have been discovered this iteration */
   NodeArray* discovered;

   /* A 2D array of int that represents the distances to the base at any given position on the map */
   int distMap[MAP_SIZE][MAP_SIZE];

   /* Member Functions */

   /**
    * @brief ExploreMap constructor.
    * @param obj Reference to self.
    * @param initX Initial x position of the drone in the map
    * @param initY Initial y position of the drone in the map
   **/
   void (*Construct) (struct _ExploreMap *obj, int initX, int initY);

   /**
    * @brief This function makes the drone move on the map.
    * @param x The x position of the node to which the drone is moved
    * @param y The x position of the node to which the drone is moved
   **/
   void (*Move) (struct _ExploreMap *obj, int x, int y);

   /**
    * @brief This function takes as input the four distance sensor values and add update the 
    * map based on thoses values.
    * @param obj Reference to self.
    * @param y_neg Front direction distance
    * @param x_pos Left direction distance
    * @param y_pos Back direction distance
    * @param x_neg Right direction distance
    * @return ??? [TODO]
   **/
   int (*AddData) (struct _ExploreMap *obj, int y_neg, int x_pos, int y_pos, int x_neg);

   /**
    * @brief This function gives the direction of exploration that would give the most information based
    * on the current exploration map.
    * @param obj Reference to self.
    * @param currDir Current exploration direction.
    * @return The direction to go to.
    */
   MapExplorationDir (*GetBestDir) (struct _ExploreMap *obj, enum _MapExplorationDir currDir);

   /**
    * @brief This function decides, based on the current location in the map, the best direction
    * the drone should move next in order to reach base in the least amount of time.
    * @param obj reference to self.
    * @param y_neg Front direction distance
    * @param x_pos Left direction distance
    * @param y_pos Back direction distance
    * @param x_neg Right direction distance
    * @return MapExplorationDir - the direction of least distance based on current location.
   **/
   MapExplorationDir (*NextNode) (struct _ExploreMap *obj, int y_neg, int x_pos, int y_pos, int x_neg);

   /**
    * @brief This function runs the Wave Propagation algorithm starting at { baseX, baseY } tile.
    * WARNING/!\ - Calling this function will overwrite every distances in the current distMap.
    * mBase must be an explored location for it to work properly. The drone must be in the map.
    * @param obj : Reference to self. 
   **/
   void (*BuildFlow) (struct _ExploreMap *obj);
} ExploreMap;

/* Global Member Functions */

extern void mConstructor (ExploreMap *obj, int initX, int initY);
extern void mMove (ExploreMap *obj, int x, int y);
extern int mAddData (ExploreMap *obj, int y_neg, int x_pos, int y_pos, int x_neg);
extern MapExplorationDir mGetBestDir (ExploreMap *obj, MapExplorationDir currDir);
extern MapExplorationDir mNextNode(ExploreMap *obj, int y_neg, int x_pos, int y_pos, int x_neg);
extern void mBuildFlowMap(ExploreMap *obj);

/* Global Functions */

/**
 * @brief Construct a ExploreMap type object. This needs to be called in order to use internal functions
 * for a desired object.
 * 
 * @param obj Reference to self.
**/
extern void ExploreMapNew(ExploreMap *obj);

#endif