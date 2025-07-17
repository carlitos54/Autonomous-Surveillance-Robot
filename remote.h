
#ifndef REMOTE_H_
#define REMOTE_H_

#include <stdint.h>
#include <stdlib.h>

extern uint32_t REMOTE;
extern uint32_t STATE;
extern uint32_t TIME;
extern uint32_t TOTAL_TIME;
extern uint32_t PREV_TIME;
extern uint32_t CODE;
extern uint32_t bits;
extern uint32_t AUTO_MODE;

void initRemote();
void Remote_ISR();

#endif
