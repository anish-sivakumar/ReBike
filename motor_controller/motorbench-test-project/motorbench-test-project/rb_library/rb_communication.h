/* 
 * File:   rb_communication.h
 * Author: siani
 *
 * Implements circular buffer for handling CAN tasks
 */

#include <stdint.h>
#include <stdbool.h>

#ifndef RB_COMMUNICATION_H
#define	RB_COMMUNICATION_H

#ifdef	__cplusplus
extern "C" {
#endif

    
#define MAX_QUEUE_SIZE 10
    
typedef enum 
{
    CAN_READ_STATUS,
    CAN_READ_RX,
    CAN_WRITE_TX
} RB_CAN_TYPE;

typedef struct 
{
    RB_CAN_TYPE type;
    uint16_t address;  // Register address for SPI
    uint16_t data;     // Data to write (if applicable)
} RB_CAN_TASK;


/** Example:
 * [0,0,0,0,0]; front=0,rear=0,size=0
 * enqueue T1: [T1,0,0,0,0]; front=0,rear=1,size=1
 * enqueue T2: [T1,T2,0,0,0]; front=0,rear=2,size=2
 * enqueue T3: [T1,T2,T3,0,0]; front=0,rear=3,size=3
 * dequeue T1: [0,T2,T3,0,0]; front=1,rear=3,size=2
 
 * front - idle when enqueueing & moves to right with dequeueing
 * rear - moves to right when enqueueing & idle when dequeueing
 */ 
typedef struct 
{
    RB_CAN_TASK tasks[MAX_QUEUE_SIZE];
    int16_t front; // index of first item 
    int16_t rear; // index of empty spot after items
    int16_t size;
} RB_CAN_QUEUE;


/**
 * Checks if queue is full according to size macro
 * @param queue
 * @return 
 */
bool isQueueFull(RB_CAN_QUEUE *queue);


/**
 * Checks if queue is empty
 * @param queue
 * @return 
 */
bool isQueueEmpty(RB_CAN_QUEUE *queue);


/**
 * Adds task into the queue at the rear position
 * @param queue
 * @param task
 * @return 
 */
bool enqueueTask(RB_CAN_QUEUE *queue, RB_CAN_TASK task);


/**
 * Removes task at the front position and records the task removed
 * @param queue
 * @param task - recording of task that is being removed 
 * @return 
 */
bool dequeueTask(RB_CAN_QUEUE *queue, RB_CAN_TASK *task);

/**
 * Runs after FOC/all other ISR tasks
 * @return 
 */
bool RB_ProcessCANTasks(RB_CAN_QUEUE *queue);

#ifdef	__cplusplus
}
#endif

#endif	/* RB_COMMUNICATION_H */

