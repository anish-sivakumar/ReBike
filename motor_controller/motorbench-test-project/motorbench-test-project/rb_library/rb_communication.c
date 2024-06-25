#include "rb_communication.h"

bool isQueueFull(RB_CAN_QUEUE *queue) {
    return queue->size == MAX_QUEUE_SIZE;
}

bool isQueueEmpty(RB_CAN_QUEUE *queue) {
    return queue->size == 0;
}

bool enqueueTask(RB_CAN_QUEUE *queue, RB_CAN_TASK task) {
    if (isQueueFull(queue)) return false;
    queue->tasks[queue->rear] = task;
    queue->rear = (queue->rear + 1) % MAX_QUEUE_SIZE;
    queue->size++;
    return true;
}

bool dequeueTask(RB_CAN_QUEUE *queue, RB_CAN_TASK *task) {
    if (isQueueEmpty(queue)) return false;
    *task = queue->tasks[queue->front];
    queue->front = (queue->front + 1) % MAX_QUEUE_SIZE;
    queue->size--;
    return true;
}


bool RB_ProcessCANTasks(RB_CAN_QUEUE *queue) {
    
    RB_CAN_TASK tempCurrentTask;
   
    /* first item is removed from queue and recorded into temp variable */
    while (dequeueTask(&queue, &tempCurrentTask)) { 
        switch (tempCurrentTask.type) {
            case SPI_READ:
                // Poll SPI register and read if available
                // Read operation logic here
                break;
            case SPI_WRITE:
                // Write data to SPI register
                // Write operation logic here
                break;
        }
        adcIsrCompleted = false;  // Ensure only one task is processed per ISR completion
    }
    
    return true; //if completed
}