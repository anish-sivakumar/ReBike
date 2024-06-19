#include "rb_board_ui.h"

#include "hal/hardware_access_functions.h"
#include "system/pins.h"
#include "adc/adc1.h"

void RB_BoardUIInit(RB_BOARD_UI* boardUI) {
    boardUI->motorEnable.state = 0;
    boardUI->potState = 0;
}

void RB_BoardUIButtonDebounce(RB_BUTTON* button, bool pressedHW) {
    // If the stored button->pressed is not equal to the hardware value, we need
    // to do debouncing
    if (pressedHW != button->pressed) {
        button->counter++;
        // If the button has been pressed for an adequate amount of cycles, then
        // change the stored button->pressed value
        if (button->counter >= RB_BUTTON_DEBOUNCE_CYCLES) {
            button->pressed = pressedHW;
            button->counter = 0;
            // If were detecting a down-press, then change the latching 
            // button->state
            if (button->pressed) {
                button->state = !button->state;
            }
        }
    } else {
        button->counter = 0;
    }
}

void RB_BoardUIService(RB_BOARD_UI* boardUI) {
    // Service motor enable button
    bool motorEnablePressedHW = !MCAF_BUTTON1_GetValue();
    RB_BoardUIButtonDebounce(&boardUI->motorEnable, motorEnablePressedHW);

    // update potentiometer value
    boardUI->potState = HAL_ADC_UnsignedFromSignedInput(ADC1_ConversionResultGet(MCAF_ADC_POTENTIOMETER));

}