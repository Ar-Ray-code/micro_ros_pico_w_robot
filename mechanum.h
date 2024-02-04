#pragma once

#include "pico.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define PICO_MAIN_CLOCK_FREQUENCY 125000000 // 125 MHz
#define PWM_STEP_MAX 1024
#define SPEED_LIMIT 0.9

typedef enum pwm_gpio_num
{
    MOTOR_A0 = 2,
    MOTOR_A1 = 3,
    MOTOR_B0 = 4,
    MOTOR_B1 = 5,
    MOTOR_C0 = 10,
    MOTOR_C1 = 11,
    MOTOR_D0 = 12,
    MOTOR_D1 = 13,
} pwm_gpio_num;

#define MOTOR_NO_INV 0
#define MOTOR_INV 1

typedef enum motor_num
{
    MOTOR_A = 0,
    MOTOR_B = 1,
    MOTOR_C = 2,
    MOTOR_D = 3,
} motor_num;


typedef struct pico_pwm_type_t
{
    uint pwm_slice_num;
    uint chan;
    uint slice_num;
} pico_pwm_type_t;


pico_pwm_type_t pico_pwm_type[] = {
    {MOTOR_A0, PWM_CHAN_A, 0},
    {MOTOR_A1, PWM_CHAN_B, 0},
    {MOTOR_B0, PWM_CHAN_A, 0},
    {MOTOR_B1, PWM_CHAN_B, 0},
    {MOTOR_C0, PWM_CHAN_A, 0},
    {MOTOR_C1, PWM_CHAN_B, 0},
    {MOTOR_D0, PWM_CHAN_A, 0},
    {MOTOR_D1, PWM_CHAN_B, 0},
};


void add_pwm_slice(pico_pwm_type_t *pico_pwm_type)
{
    gpio_set_function(pico_pwm_type->pwm_slice_num, GPIO_FUNC_PWM);
    uint pwm_slice_num = pwm_gpio_to_slice_num(pico_pwm_type->pwm_slice_num);
    pico_pwm_type->slice_num = pwm_slice_num;
    pwm_set_clkdiv(pwm_slice_num, (float)(PICO_MAIN_CLOCK_FREQUENCY / (10000 * PWM_STEP_MAX)));
    pwm_set_wrap(pwm_slice_num, PWM_STEP_MAX);
    pwm_set_chan_level(pwm_slice_num, pico_pwm_type->chan, 0);
    pwm_set_enabled(pwm_slice_num, true);
}

void set_pwm_duty(pico_pwm_type_t *pico_pwm_type, float duty_percent)
{
    (duty_percent > 1) ? (duty_percent = 1) : (duty_percent = duty_percent);
    (duty_percent < 0) ? (duty_percent = 0) : (duty_percent = duty_percent);
    pwm_set_chan_level(pico_pwm_type->slice_num, pico_pwm_type->chan, (uint)(PWM_STEP_MAX * duty_percent));
}

// --------------------------------------------
void set_motor(motor_num motor, float duty_percent, bool invert)
{
    if (invert)
    {
        duty_percent = -duty_percent;
    }

    (duty_percent > SPEED_LIMIT) ? (duty_percent = SPEED_LIMIT) : (duty_percent = duty_percent);
    (duty_percent < -SPEED_LIMIT) ? (duty_percent = -SPEED_LIMIT) : (duty_percent = duty_percent);

    if (duty_percent > 0)
    {
        set_pwm_duty(&pico_pwm_type[motor * 2], duty_percent);
        set_pwm_duty(&pico_pwm_type[motor * 2 + 1], 0);
    }
    else if (duty_percent < 0)
    {
        set_pwm_duty(&pico_pwm_type[motor * 2], 0);
        set_pwm_duty(&pico_pwm_type[motor * 2 + 1], -duty_percent);
    }
    else
    {
        set_pwm_duty(&pico_pwm_type[motor * 2], 0);
        set_pwm_duty(&pico_pwm_type[motor * 2 + 1], 0);
    }
}

void add_pwm_slice_all()
{
    for (int i = 0; i < 8; i++)
    {
        add_pwm_slice(&pico_pwm_type[i]);
    }
}

void omni(float x, float y, float w)
{
    set_motor(MOTOR_A, y + x + w, MOTOR_NO_INV);
    set_motor(MOTOR_B, y - x - w, MOTOR_NO_INV);
    set_motor(MOTOR_C, y - x + w, MOTOR_NO_INV);
    set_motor(MOTOR_D, y + x - w, MOTOR_INV);
}

void stop()
{
    set_motor(MOTOR_A, 0, MOTOR_NO_INV);
    set_motor(MOTOR_B, 0, MOTOR_NO_INV);
    set_motor(MOTOR_C, 0, MOTOR_NO_INV);
    set_motor(MOTOR_D, 0, MOTOR_INV);
}