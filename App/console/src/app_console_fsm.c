#include <console/inc/app_console.h>
#include <console/inc/app_console_fsm.h>
#include <led/inc/app_led.h>
#include <stdio.h>

#define APP_CONSOLE_STDIN_BUF_SIZE 16U

static void _fsm_go_to_main_menu(void *event_data);
static void _fsm_go_to_led_menu(void *event_data);
static void _fsm_main_menu_cmd_reset(void *event_data);
static void _fsm_led_menu_get_pwm_mode(void *event_data);
static void _fsm_led_menu_set_pwm_mode(void *event_data);
static void _fsm_led_menu_get_brightness(void *event_data);
static void _fsm_led_menu_set_brightness(void *event_data);

static const char *main_menu_options[] =
{
    [APP_CONSOLE_OPTION_0] = "Reset",
    [APP_CONSOLE_OPTION_1] = "LED control menu"
};
static const size_t main_menu_options_count = sizeof(main_menu_options)  / sizeof(main_menu_options[0]);

static const char *led_menu_options[] =
{
    [APP_CONSOLE_OPTION_0] = "LED get mode",
    [APP_CONSOLE_OPTION_1] = "LED set mode",
    [APP_CONSOLE_OPTION_2] = "LED get brightness",
    [APP_CONSOLE_OPTION_3] = "LED set brightness",
    [APP_CONSOLE_OPTION_4] = "Back",
    [APP_CONSOLE_OPTION_5] = "Main menu"
};
static const size_t led_menu_options_count = sizeof(led_menu_options)  / sizeof(led_menu_options[0]);

struct
{
    const char *menu_name;
    const size_t options_count;
    const char **options_desc_tbl;
} static const menus_desc_tbl[] =
{
    [APP_CONSOLE_MAIN_MENU] = {"Main menu",        main_menu_options_count, main_menu_options},
    [APP_CONSOLE_LED_MENU]  = {"LED control menu", led_menu_options_count,  led_menu_options}
};
static const size_t menus_desc_tbl_size = sizeof(menus_desc_tbl) / sizeof(menus_desc_tbl[0]);

const fsm_transition_t app_console_fsm_transitions[APP_CONSOLE_MENUS_COUNT][APP_CONSOLE_OPTIONS_COUNT] =
{
    [APP_CONSOLE_NO_MENU] =
    {
        {APP_CONSOLE_MAIN_MENU, &_fsm_go_to_main_menu}
    },
    [APP_CONSOLE_MAIN_MENU] =
    {
        [APP_CONSOLE_OPTION_0] = {APP_CONSOLE_MAIN_MENU, &_fsm_main_menu_cmd_reset},
        [APP_CONSOLE_OPTION_1] = {APP_CONSOLE_LED_MENU,  &_fsm_go_to_led_menu},
        [APP_CONSOLE_OPTION_2] = FSM_NO_TRANSITION,
        [APP_CONSOLE_OPTION_3] = FSM_NO_TRANSITION,
        [APP_CONSOLE_OPTION_4] = FSM_NO_TRANSITION,
        [APP_CONSOLE_OPTION_5] = FSM_NO_TRANSITION,
        [APP_CONSOLE_OPTION_6] = FSM_NO_TRANSITION,
        [APP_CONSOLE_OPTION_7] = FSM_NO_TRANSITION,
        [APP_CONSOLE_OPTION_8] = FSM_NO_TRANSITION,
        [APP_CONSOLE_OPTION_9] = FSM_NO_TRANSITION
    },
    [APP_CONSOLE_LED_MENU] =
    {
        [APP_CONSOLE_OPTION_0] = {APP_CONSOLE_LED_MENU,  &_fsm_led_menu_get_pwm_mode},
        [APP_CONSOLE_OPTION_1] = {APP_CONSOLE_LED_MENU,  &_fsm_led_menu_set_pwm_mode},
        [APP_CONSOLE_OPTION_2] = {APP_CONSOLE_LED_MENU,  &_fsm_led_menu_get_brightness},
        [APP_CONSOLE_OPTION_3] = {APP_CONSOLE_LED_MENU,  &_fsm_led_menu_set_brightness},
        [APP_CONSOLE_OPTION_4] = {APP_CONSOLE_MAIN_MENU, &_fsm_go_to_main_menu},
        [APP_CONSOLE_OPTION_5] = {APP_CONSOLE_MAIN_MENU, &_fsm_go_to_main_menu},
        [APP_CONSOLE_OPTION_6] = FSM_NO_TRANSITION,
        [APP_CONSOLE_OPTION_7] = FSM_NO_TRANSITION,
        [APP_CONSOLE_OPTION_8] = FSM_NO_TRANSITION,
        [APP_CONSOLE_OPTION_9] = FSM_NO_TRANSITION
    }
};

static void _propose_options_for_menu(app_console_menu_t menu)
{
    if (menu < menus_desc_tbl_size)
    {
        printf("%s\n", menus_desc_tbl[menu].menu_name);

        printf("Options:\n");

        size_t opt = 0;
        for(; opt < menus_desc_tbl[menu].options_count; opt++)
            printf("%d: %s\n", opt, menus_desc_tbl[menu].options_desc_tbl[opt]);

        printf("Enter option(0-9): \n");
    }
}

static inline char *_get_line_from_stdin(char *buf, int size)
{
    return fgets(buf, size, stdin);
}

static int _get_int_from_stdin()
{
    int ret = EOF;

    char buf[APP_CONSOLE_STDIN_BUF_SIZE*2] = {0};

    if (_get_line_from_stdin(buf, sizeof(buf)))
        sscanf(buf, "%d\n", &ret);

    return ret;
}

static void _fsm_go_to_main_menu(void *event_data)
{
    (void)event_data;

    printf("Welcome to the console app v"APP_CONSOLE_VERSION"!\n");

    _propose_options_for_menu(APP_CONSOLE_MAIN_MENU);
}

static void _fsm_go_to_led_menu(void *event_data)
{
    (void)event_data;

    _propose_options_for_menu(APP_CONSOLE_LED_MENU);
}

static void _fsm_main_menu_cmd_reset(void *event_data)
{
    (void)event_data;

    HAL_NVIC_SystemReset();
}

static void _fsm_led_menu_get_pwm_mode(void *event_data)
{
    (void)event_data;

    app_led_pwm_mode_t led_pwm_mode = app_led_get_pwm_mode();
    printf("LED PWM mode is: %s (%d)\n", app_led_pwm_mode_to_str(led_pwm_mode), led_pwm_mode);

    _propose_options_for_menu(APP_CONSOLE_LED_MENU);
}

static void _fsm_led_menu_set_pwm_mode(void *event_data)
{
    (void)event_data;

    printf("PWM LED modes:\n");

    app_led_pwm_mode_t mode = APP_LED_PWM_MODE_FIRST;
    for (; mode <= APP_LED_PWM_MODE_LAST; mode++)
        printf("%d - %s\n", mode, app_led_pwm_mode_to_str(mode));

    printf("Please enter mode number(%d - %d):\n", APP_LED_PWM_MODE_FIRST, APP_LED_PWM_MODE_LAST);

    mode = (app_led_pwm_mode_t)_get_int_from_stdin();
    if (mode >= 0)
    {
        if ((mode >= APP_LED_PWM_MODE_FIRST) && (mode <= APP_LED_PWM_MODE_LAST))
        {
            if (!app_led_set_pwm_mode(mode))
                printf("Error setting LED PWM mode!\n");
        }
        else
        {
            printf("Wrong LED PWM mode entered: %d, required to be in range from %d to %d\n", mode, APP_LED_PWM_MODE_FIRST, APP_LED_PWM_MODE_LAST);
        }
    }
    else
        printf("Incorrect mode value entered!\n");

    _propose_options_for_menu(APP_CONSOLE_LED_MENU);
}

static void _fsm_led_menu_get_brightness(void *event_data)
{
    (void)event_data;

    printf("LED brightness is: %d\n", app_led_get_brightness());

    _propose_options_for_menu(APP_CONSOLE_LED_MENU);
}

static void _fsm_led_menu_set_brightness(void *event_data)
{
    (void)event_data;

    printf("Enter LED brightness (0-255): \n");

    int brightness = _get_int_from_stdin();

    if ((brightness >= 0) && (brightness <= APP_LED_BRIGHTNESS_MAX))
        app_led_set_brightness((uint8_t)brightness);
    else
        printf("Incorrect brightness level entered: %d, expected to be in range: 0 - 255.\n", brightness);

    _propose_options_for_menu(APP_CONSOLE_LED_MENU);
}