#ifndef NV_PWM_H
#define NV_PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ESC/PWM 출력 채널 수 (쿼드콥터) */
#define NV_PWM_CHANNELS 4
#define NV_PWM_MIN_US 1000
#define NV_PWM_MAX_US 2000

typedef struct {
    uint16_t pulse_us[NV_PWM_CHANNELS];
    uint8_t armed;
} nv_pwm_state_t;

void nv_pwm_init(nv_pwm_state_t *state);
void nv_pwm_arm(nv_pwm_state_t *state);
void nv_pwm_disarm(nv_pwm_state_t *state);
/* percent: 0.0 ~ 100.0 */
int nv_pwm_set_throttle(nv_pwm_state_t *state, uint8_t channel, float percent);
uint16_t nv_pwm_pulse_us(const nv_pwm_state_t *state, uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* NV_PWM_H */
