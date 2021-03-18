#ifndef EXPLORE_MAP_H
#define EXPLORE_MAP_H

typedef struct _ExploreMap {
    int currX;
    int currY;
    uint8_t mapResolutionCM;
    uint8_t map[30][30];
    /* Member Functions */
    void (*Construct) (struct _ExploreMap *obj, int initX, int initY, int arenaSize);
    void (*Move) (struct _ExploreMap *obj, int x, int y);
    void (*AddData) (struct _ExploreMap *obj, int frontDist, int leftDist, int backDist, int rightDist);
} ExploreMap;

/* Global Member Functions */
extern void mConstructor (ExploreMap *obj, int initX, int initY, int arenaSize);
extern void mMove (ExploreMap *obj, int x, int y);
extern void mAddData (ExploreMap *obj, int frontDist, int leftDist, int backDist, int rightDist);

/* Global Functions */
extern void ExploreMapNew(ExploreMap *obj);

#endif