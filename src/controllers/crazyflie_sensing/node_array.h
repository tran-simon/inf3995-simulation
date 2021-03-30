#ifndef NODE_ARRAY_H
#define NODE_ARRAY_H

#include "node.h"
#include <stddef.h>
/**
 * @brief NodeArray is a dynamicaly allocated array of Nodes.
 */
typedef struct _NodeArray {
    Node *array;
    size_t used;
    size_t size;

    /* Member Functions */
    void (*Init) (struct _NodeArray *a, size_t initialSize);
    void (*Insert) (struct _NodeArray *a, Node element);
    void (*Free) (struct _NodeArray *a);

} NodeArray;

/* Global Member Functions */

/**
 * @brief Initialize a NodeArray type object.
 * 
 * @param a : reference to self. Give current NodeArray as a reference parameter.
 * @param initialSize : number of NodeArray elements expected to be stored in the array.
 * If the array is full, the size will double automaticatily in order to prevent overflow.
 */
extern void mInit(NodeArray *a, size_t initialSize);

/**
 * @brief Insert a Node type object into a Node type array. If the Node added already exist in
 * the array, it won't be added in order to prevent duplication.
 * 
 * @param a reference to self. Give current NodeArray as a reference parameter.
 * @param element The Node type object to add in the array.
 */
extern void mInsert(NodeArray *a, Node element);

/**
 * @brief Free memory allocated to the NodeArray.
 * 
 * @param a reference to self. Give current NodeArray as a reference parameter.
 */
extern void mFree(NodeArray *a); 

/* Global Functions */
/**
 * @brief Construct a NodeArray type object. This needs to be called in order to use internal function
 * for a desired object.
 * 
 * @param a reference to self. Give new NodeArray as a reference parameter.
 */
extern void NodeArrayNew(NodeArray *a);

#endif