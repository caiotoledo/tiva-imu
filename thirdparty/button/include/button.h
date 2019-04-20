#ifndef BUTTON_H_
#define BUTTON_H_

typedef enum {
    ButtonSW1,
    ButtonSW2,
} eButtonSW;

typedef uint32_t (*millis)();

void Button_Enable(millis f);
bool Button_Get_State(eButtonSW b);


#endif /* BUTTON_H_ */
