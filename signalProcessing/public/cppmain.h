//
// Created by vis75817 on 1/12/2022.
//

#ifndef MCU2_CPPMAIN_H
#define MCU2_CPPMAIN_H
#include "main.h"
#ifdef __cplusplus
extern "C" {
#endif

void my_systick_Callback(void);

#ifdef __cplusplus
[[noreturn]]
#endif
void cppmain(struct p_HW hardware);

    enum Event{
        kTick,
        kDebug,
        kMotorUpdate,
        kTimer14
    };
    void push_message(enum Event);

#ifdef __cplusplus
} // extern "C"
#endif

#endif //MCU2_CPPMAIN_H
