#ifndef __QUEUE_AAR_H__
#define __QUEUE_AAR_H__

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>
// A structure to represent a queue
struct Queue
{
    int front, rear, size;
    unsigned capacity;
    uint8_t* array;
};
 

 /*Function Prototype*/
struct Queue* createQueue(unsigned capacity);
void enqueue(struct Queue* queue, uint8_t item);
uint8_t dequeue(struct Queue* queue);
int isFull(struct Queue* queue);
int isEmpty(struct Queue* queue);

#endif
