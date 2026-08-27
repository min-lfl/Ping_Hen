#ifndef __CONTROL_CONFIG_H__
#define __CONTROL_CONFIG_H__

/* Sampling rates must match TIM1 and TIM2. */
#define CONTROL_IMU_SAMPLE_HZ             300.0f
#define CONTROL_LOOP_HZ                   100.0f
#define CONTROL_AUTOMATIC_ENABLE          (1)

/* Motor driver limits. ACC=0 means direct start and should not be used here. */
#define CONTROL_MOTOR_SPEED_RPM           (100)
#define CONTROL_MOTOR_ACCEL_PARAM         (60)
#define CONTROL_MANUAL_TEST_PULSE         (250)

/* Accelerometer calibration. At rest: corrected Ay should be 0 g. */
#define CONTROL_ACCEL_Y_BIAS_G            (-0.001532f)
#define CONTROL_ACCEL_Z_BIAS_G            (0.0f)
#define CONTROL_ACCEL_TO_ROD_SIGN         (-1.0f)
#define CONTROL_ACCEL_INPUT_LIMIT_G       (0.30f)

/* Two cascaded first-order poles. Raise for less delay, lower for less shake. */
#define CONTROL_ACCEL_LPF_POLE_HZ         (22.0f)

/* Absolute motor zero and safe linkage travel, in motor pulses. */
#define CONTROL_LEVEL_PULSE               (0)
#define CONTROL_MIN_RELATIVE_PULSE        (-250)
#define CONTROL_MAX_RELATIVE_PULSE        (250)

/* Software S-curve limits, in motor pulse units. */
#define CONTROL_PROFILE_NATURAL_HZ        (3.0f)
#define CONTROL_PROFILE_DAMPING           (1.0f)
#define CONTROL_MAX_PULSE_SPEED           (2500.0f)   /* pulse/s */
#define CONTROL_MAX_PULSE_ACCEL           (30000.0f)  /* pulse/s^2 */
#define CONTROL_MAX_PULSE_JERK            (1000000.0f) /* pulse/s^3 */

/* Do not resend tiny position changes. The motor holds the last target. */
#define CONTROL_SEND_HYSTERESIS_PULSE     (3)
#define CONTROL_SETTLE_POSITION_PULSE     (0.35f)
#define CONTROL_SETTLE_SPEED_PULSE_S      (5.0f)
#define CONTROL_SETTLE_ACCEL_PULSE_S2     (100.0f)

#endif
