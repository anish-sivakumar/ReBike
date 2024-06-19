/* 
 * File:   rb_board_ui.h
 * Author: Chris Hyggen
 *
 * Created on June 15, 2024, 12:55 PM
 */

#ifndef RB_BOARD_UI_H
#define	RB_BOARD_UI_H

#include "stdbool.h"
#include "stdint.h"


#define RB_BUTTON_DEBOUNCE_CYCLES 100

typedef struct tagRB_BUTTON {
    bool state; // latching
    bool pressed; // non-latching 
    uint16_t counter;
} RB_BUTTON;

typedef struct tagRB_BOARD_UI {
    RB_BUTTON motorEnable;
    uint16_t potState;
} RB_BOARD_UI;

void RB_BoardUIInit(RB_BOARD_UI* boardUI);

void RB_BoardUIService(RB_BOARD_UI* boardUI);


#endif	/* RB_BOARD_UI_H */

