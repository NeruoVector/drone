#include "nv_pwm.h"

#include <stddef.h>

void nv_pwm_init(nv_pwm_state_t *state) {
    if (state == NULL) {
        return;
    }
    for (size_t i = 0; i < NV_PWM_CHANNELS; ++i) {
        state->pulse_us[i] = NV_PWM_MIN_US;
    }
    state->armed = 0;
}

void nv_pwm_arm(nv_pwm_state_t *state) {
    if (state == NULL) {
        return;
    }
    state->armed = 1;
}

void nv_pwm_disarm(nv_pwm_state_t *state) {
    if (state == NULL) {
        return;
    }
    state->armed = 0;
    for (size_t i = 0; i < NV_PWM_CHANNELS; ++i) {
        state->pulse_us[i] = NV_PWM_MIN_US;
    }
}

int nv_pwm_set_throttle(nv_pwm_state_t *state, uint8_t channel, float percent) {
    if (state == NULL || channel >= NV_PWM_CHANNELS) {
        return -1;
    }
    /* 시동이 걸리지 않은 상태에서는 출력을 변경하지 않는다. */
    if (!state->armed) {
        return -2;
    }
    if (percent < 0.0f) {
        percent = 0.0f;
    }
    if (percent > 100.0f) {
        percent = 100.0f;
    }

    const float span = (float)(NV_PWM_MAX_US - NV_PWM_MIN_US);
    state->pulse_us[channel] = (uint16_t)(NV_PWM_MIN_US + span * (percent / 100.0f));
    return 0;
}

uint16_t nv_pwm_pulse_us(const nv_pwm_state_t *state, uint8_t channel) {
    if (state == NULL || channel >= NV_PWM_CHANNELS) {
        return NV_PWM_MIN_US;
    }
    return state->pulse_us[channel];
}
