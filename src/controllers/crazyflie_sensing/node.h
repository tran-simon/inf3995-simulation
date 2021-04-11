#ifndef NODE_H
#define NODE_H

/**
 * Node define a tile in the Wave propagation exploration algorithm.
 * @param distance : represents the distance relative to neighbour tiles. This value
 * is modified by the wave propagation function and must not be altered with unless the 
 * destination has changed.
 * @param status : represents the state of a Node. State 0 : invalid (unexplored or obstructed)
 * State 1 : valid   (explored and free)
 */
struct Node {
    int x;
    int y;
    int distance;
    int status;
};

#endif