#include <display/inc/max7219_led_matrix.h>
#include <stdbool.h>

#define MAX7219_SPI_DATA_SIZE ((uint16_t)(2U))
#define CS_PIN_INVALID_VALUE ((uint16_t)-1)

typedef enum max7219_reg_addr_e
{
    MAX7219_REG_ADDR_NO_OP = 0x0,
    MAX7219_REG_ADDR_DIGIT_0 = 0x1,
    MAX7219_REG_ADDR_DIGIT_1 = 0x2,
    MAX7219_REG_ADDR_DIGIT_2 = 0x3,
    MAX7219_REG_ADDR_DIGIT_3 = 0x4,
    MAX7219_REG_ADDR_DIGIT_4 = 0x5,
    MAX7219_REG_ADDR_DIGIT_5 = 0x6,
    MAX7219_REG_ADDR_DIGIT_6 = 0x7,
    MAX7219_REG_ADDR_DIGIT_7 = 0x8,
    MAX7219_REG_ADDR_DECODE_MODE = 0x9,
    MAX7219_REG_ADDR_INTENSITY = 0xA,
    MAX7219_REG_ADDR_SCAN_LIMIT = 0xB,
    MAX7219_REG_ADDR_SHUTDOWN = 0xC,
    MAX7219_REG_ADDR_DISPLAY_TEST = 0xF,
} max7219_reg_addr_t;

typedef enum max7219_shutdown_mode_e
{
    MAX7219_SHUTDOWN_MODE_SHUTDOWN = 0x0,
    MAX7219_SHUTDOWN_MODE_NORMAL_OPERATION = 0x1
} max7219_shutdown_mode_t;

typedef enum max7219_decode_mode_e
{
    MAX7219_DECODE_MODE_NO_DECODE = 0x0,
    MAX7219_DECODE_MODE_CODE_B_0_NO_DECODE_1_7 = 0x1,
    MAX7219_DECODE_MODE_CODE_B_0_3_NO_DECODE_4_7 = 0x0F,
    MAX7219_DECODE_MODE_CODE_B = 0xFF
} max7219_decode_mode_t;

typedef enum max7219_intensity_e
{
    MAX7219_INTENSITY_MIN = 0x0,
    MAX7219_INTENSITY_9_PERCENT = 0x1,
    MAX7219_INTENSITY_15_PERCENT = 0x2,
    MAX7219_INTENSITY_21_PERCENT = 0x3,
    MAX7219_INTENSITY_28_PERCENT = 0x4,
    MAX7219_INTENSITY_34_PERCENT = 0x5,
    MAX7219_INTENSITY_40_PERCENT = 0x6,
    MAX7219_INTENSITY_46_PERCENT = 0x7,
    MAX7219_INTENSITY_53_PERCENT = 0x8,
    MAX7219_INTENSITY_59_PERCENT = 0x9,
    MAX7219_INTENSITY_65_PERCENT = 0xA,
    MAX7219_INTENSITY_71_PERCENT = 0xB,
    MAX7219_INTENSITY_78_PERCENT = 0xC,
    MAX7219_INTENSITY_84_PERCENT = 0xD,
    MAX7219_INTENSITY_90_PERCENT = 0xE,
    MAX7219_INTENSITY_MAX = 0xF
} max7219_intensity_t;

typedef enum max7219_scan_limit_e
{
    MAX7219_SCAN_LIMIT_DIGIT_0_ONLY = 0x0,
    MAX7219_SCAN_LIMIT_DIGITS_0_AND_1 = 0x1,
    MAX7219_SCAN_LIMIT_DIGITS_0_TO_2 = 0x2,
    MAX7219_SCAN_LIMIT_DIGITS_0_TO_3 = 0x3,
    MAX7219_SCAN_LIMIT_DIGITS_0_TO_4 = 0x4,
    MAX7219_SCAN_LIMIT_DIGITS_0_TO_5 = 0x5,
    MAX7219_SCAN_LIMIT_DIGITS_0_TO_6 = 0x6,
    MAX7219_SCAN_LIMIT_ALL_DIGITS = 0x7
} max7219_scan_limit_t;

typedef enum max7219_display_test_e
{
    MAX7219_DISPLAY_TEST_NORMAL_OPERATION = 0x0,
    MAX7219_DISPLAY_TEST_MODE = 0x1
} max7219_display_test_t;

/* typedef union max7219_reg_data_u
{
    max7219_shutdown_mode_t shutdown_mode;
    max7219_decode_mode_t decode_mode;
    max7219_intensity_t intensity;
    max7219_scan_limit_t scan_limit;
    max7219_display_test_t display_test;
    max7219_led_matrix_segment_t segment_bitmask;
} max7219_reg_data_t; */

static HAL_StatusTypeDef max7219_spi_send(max7219_reg_addr_t reg_addr, uint8_t data);

static SPI_HandleTypeDef *_spi_handle = NULL;
static GPIO_TypeDef *_cs_gpio_port = NULL;
static uint16_t _cs_pin = CS_PIN_INVALID_VALUE;

static inline HAL_StatusTypeDef is_led_matrix_initialized()
{
    return (_spi_handle && _cs_gpio_port && (_cs_pin != CS_PIN_INVALID_VALUE)) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef led_matrix_init(SPI_HandleTypeDef *spi_handle, GPIO_TypeDef *cs_gpio_port, uint16_t cs_pin)
{
    HAL_StatusTypeDef result = HAL_OK;

    do
    {
        if (!spi_handle || !cs_gpio_port)
        {
            result = HAL_ERROR;
            break;
        }

        _spi_handle = spi_handle;
        _cs_gpio_port = cs_gpio_port;
        _cs_pin = cs_pin;

        const uint8_t max7219_init_data[][MAX7219_SPI_DATA_SIZE] =
        {
            {MAX7219_REG_ADDR_SHUTDOWN,     MAX7219_SHUTDOWN_MODE_NORMAL_OPERATION},
            {MAX7219_REG_ADDR_DECODE_MODE,  MAX7219_DECODE_MODE_NO_DECODE},
            {MAX7219_REG_ADDR_INTENSITY,    MAX7219_INTENSITY_40_PERCENT},
            {MAX7219_REG_ADDR_SCAN_LIMIT,   MAX7219_SCAN_LIMIT_ALL_DIGITS},
            {MAX7219_REG_ADDR_DISPLAY_TEST, MAX7219_DISPLAY_TEST_NORMAL_OPERATION}
        };

        size_t i = 0;
        for (; i < sizeof(max7219_init_data) / sizeof(max7219_init_data[0]); i++)
        {
            result = max7219_spi_send(max7219_init_data[i][0], max7219_init_data[i][1]);

            if (result != HAL_OK)
                break;
        }

        if (result != HAL_OK)
            break;

        result = led_matrix_clear();
    }
    while (false);

    return result;
}

HAL_StatusTypeDef led_matrix_draw_bitmap(uint8_t bitmap[MAX_7219_LED_MATRIX_SIZE])
{
    HAL_StatusTypeDef result = is_led_matrix_initialized();

    if (result == HAL_OK)
    {
        result = led_matrix_clear();
        if (result == HAL_OK)
        {
            size_t i = 0;
            for (; i < MAX_7219_LED_MATRIX_SIZE; i++)
            {
                result = max7219_spi_send(MAX7219_REG_ADDR_DIGIT_0 + i, bitmap[i]); // TODO: rotate bitmap in order to send proper bitmap, or find some other way
                if (result != HAL_OK)
                    break;
            }
        }
    }

    return result;
}

HAL_StatusTypeDef led_matrix_clear()
{
    HAL_StatusTypeDef result = is_led_matrix_initialized();

    if (result == HAL_OK)
    {
        max7219_reg_addr_t digit_reg_addr = MAX7219_REG_ADDR_DIGIT_0;
        for (; digit_reg_addr <= MAX7219_REG_ADDR_DIGIT_7; digit_reg_addr++)
        {
            result = max7219_spi_send(digit_reg_addr, (uint8_t)MAX7219_LED_MATRIX_NO_SEGMENT);

            if (result != HAL_OK)
                break;
        }
    }

    return result;
}

static HAL_StatusTypeDef max7219_spi_send(max7219_reg_addr_t reg_addr, uint8_t data)
{
    HAL_StatusTypeDef result = is_led_matrix_initialized();

    if (result == HAL_OK)
    {
        HAL_GPIO_WritePin(_cs_gpio_port, _cs_pin, GPIO_PIN_RESET);
    
        uint8_t buffer[MAX7219_SPI_DATA_SIZE] = {reg_addr, (uint8_t)data};
        result = HAL_SPI_Transmit(_spi_handle, (const uint8_t*)buffer, sizeof(buffer), 1000);
    
        HAL_GPIO_WritePin(_cs_gpio_port, _cs_pin, GPIO_PIN_SET);
    }

    return result;
}