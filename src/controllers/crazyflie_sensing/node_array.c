#include "node_array.h"
#include <argos3/core/utility/math/general.h>
#include <stdlib.h>

extern void NodeArrayNew(NodeArray *a) {
    a->Init   = mInit;
    a->Insert = mInsert;
    a->Free   = mFree;
}

extern void mInit(NodeArray *a, size_t initialSize) {
    a->array = (Node *)malloc(initialSize * sizeof(Node));
    a->used = 0;
    a->size = initialSize;
}

extern void mInsert(NodeArray *a, Node element) {
    char cmpChar = '0';
    if(a->used != 0) {
        for(int i = 0; i < a->used; i++) {
            if(a->array[i].x == element.x && a->array[i].y  == element.y && a->array[i].distance == element.distance) {
                a->array[i].distance = argos::Min(a->array[i].distance, element.distance);
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

extern void mFree(NodeArray *a) {
    free(a->array);
    a->array = NULL;
    a->used = a->size = 0;
}