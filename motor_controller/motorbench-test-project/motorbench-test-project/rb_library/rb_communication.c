#include "rb_communication.h"

bool isQueueFull(RB_CAN_QUEUE *queue) 
{
    return queue->size == MAX_QUEUE_SIZE;
}

bool isQueueEmpty(RB_CAN_QUEUE *queue) 
{
    return queue->size == 0;
}

bool enqueueTask(RB_CAN_QUEUE *queue, RB_CAN_TASK task) 
{
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


bool RB_ProcessCANTasks(RB_CAN_QUEUE *pqueue) 
{
    
    RB_CAN_TASK tempCurrentTask;
   
    /* first item is removed from queue and recorded into temp variable */
    while (dequeueTask(pqueue, &tempCurrentTask)) { 
        switch (tempCurrentTask.type) {
            case CAN_READ_STATUS:
                // Is there something to read?
                break;
            case CAN_READ_RX:
                // Read the data from Rx buffer
                // parse CAN frame
                break;
            case CAN_WRITE_TX:
                // build the CAN frame
                // write data to Tx buffer
                break;
        }
        
    }
    
    return true; //if completed
}