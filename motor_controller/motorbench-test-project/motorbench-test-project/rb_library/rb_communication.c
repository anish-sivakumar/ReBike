#include "rb_communication.h"

bool RB_ProcessCANTasks(RB_CAN_QUEUE *pqueue) 
{
    
    RB_CAN_TASK tempCurrentTask;
   
    /* first item is removed from queue and recorded into temp variable */
    while (RB_DequeueTask(pqueue, &tempCurrentTask)) { 
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