// C program for array implementation of queue
#include "queue_aar.h"
#include "stm32f4xx_hal.h"

// function to create a queue of given capacity. 
// It initializes size of queue as 0
struct Queue* createQueue(unsigned capacity)
{
    struct Queue* queue = (struct Queue*) malloc(sizeof(struct Queue));
    queue->capacity = capacity;
    queue->front = queue->size = 0; 
    queue->rear = capacity - 1;  // This is important, see the enqueue
    queue->array = (uint8_t*) calloc(queue->capacity,queue->capacity * sizeof(uint8_t));
    return queue;
}
 
// Queue is full when size becomes equal to the capacity 
int isFull(struct Queue* queue)
{  return (queue->size == queue->capacity);  }
 
// Queue is empty when size is 0
int isEmpty(struct Queue* queue)
{  return (queue->size == 0); }
 
// Function to add an item to the queue.  
// It changes rear and size
/*Changed from Pointer to Uint8_t */
void enqueue(struct Queue* queue, uint8_t item)
{
	//__disable_irq();
    if (isFull(queue))
    {
    	//__enable_irq();
        return;
    }
    queue->rear = (queue->rear + 1)%queue->capacity;
    queue->array[queue->rear] = item;
    queue->size = queue->size + 1;
    //__enable_irq();
    //printf("%x enqueued to queue\r\n", item);
}
 
// Function to remove an item from queue. 
// It changes front and size
uint8_t dequeue(struct Queue* queue)
{
	//__disable_irq();
    if (isEmpty(queue))
    {
    	//__enable_irq();
        return INT_MIN;
    }
    char data = queue->array[queue->front];
    queue->front = (queue->front + 1)%queue->capacity;
    queue->size = queue->size - 1;
    //__enable_irq();
    return data;
}
 
// Function to get front of queue
int front(struct Queue* queue)
{
    if (isEmpty(queue))
        return INT_MIN;
    return queue->array[queue->front];
}
 
// Function to get rear of queue
int rear(struct Queue* queue)
{
    if (isEmpty(queue))
        return INT_MIN;
    return queue->array[queue->rear];
}
