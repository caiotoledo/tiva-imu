#ifndef BUTTON_H_
#define BUTTON_H_

typedef enum {
    ButtonSW1,
    ButtonSW2,
} eButtonSW;

/**
 * @brief Enable all buttons GPIOs.
 * 
 * @param func Pointer function to provide the ms counter
 */
void Button_Enable(uint32_t (*func)());

/**
 * @brief Get the button state.
 * 
 * @param b Button desired
 * @return true Button pressed
 * @return false Button released
 */
bool Button_Get_State(eButtonSW b);


#endif /* BUTTON_H_ */
