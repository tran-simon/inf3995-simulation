#ifndef EXPLORE_MAP_H
#define EXPLORE_MAP_H

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
    uint8_t mapResolutionCM;
    uint8_t map[50][50];
    /* Member Functions */
    void (*Construct) (struct _ExploreMap *obj, int initX, int initY);
    void (*Move) (struct _ExploreMap *obj, int x, int y);
    int (*AddData) (struct _ExploreMap *obj, int y_neg, int x_pos, int y_pos, int x_neg);
    MapExplorationDir (*GetBestDir) (struct _ExploreMap *obj);
} ExploreMap;

/* Global Member Functions */
extern void mConstructor (ExploreMap *obj, int initX, int initY);
extern void mMove (ExploreMap *obj, int x, int y);
extern int mAddData (ExploreMap *obj, int y_neg, int x_pos, int y_pos, int x_neg);
extern MapExplorationDir mGetBestDir (ExploreMap *obj);

/* Global Functions */
extern void ExploreMapNew(ExploreMap *obj);

#endif