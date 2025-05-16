#ifndef APP_CONSOLE_FSM_H
#define APP_CONSOLE_FSM_H

#include <lib/simple_state_machine/include/simple_fsm_v2.h>

typedef enum
{
    APP_CONSOLE_MAIN_MENU,
    APP_CONSOLE_LED_MENU,

    APP_CONSOLE_NO_MENU,
    APP_CONSOLE_MENUS_COUNT
} app_console_menu_t;

typedef enum
{
    APP_CONSOLE_OPTION_0,
    APP_CONSOLE_OPTION_1,
    APP_CONSOLE_OPTION_2,
    APP_CONSOLE_OPTION_3,
    APP_CONSOLE_OPTION_4,
    APP_CONSOLE_OPTION_5,
    APP_CONSOLE_OPTION_6,
    APP_CONSOLE_OPTION_7,
    APP_CONSOLE_OPTION_8,
    APP_CONSOLE_OPTION_9,

    APP_CONSOLE_OPTIONS_COUNT
} app_console_option_t;

extern const fsm_transition_t app_console_fsm_transitions[APP_CONSOLE_MENUS_COUNT][APP_CONSOLE_OPTIONS_COUNT];

#endif