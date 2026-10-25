#include "os.h"
#include "motor_tt.h"
#include "pwm.h"
#include "pid.h"
#include "sbus.h"

#include <math.h>

#define ctrl_period_ms 10u
#define stick_deadzone 0.05f
#define duty_limit 1.0f

#define SIGN_LF  (+1.0f)
#define SIGN_RF  (-1.0f)
#define SIGN_LB  (+1.0f)
#define SIGN_RB  (+1.0f)

#define ROLLER_MOTOR   MOTOR_TT_3
#define LIFT_MOTOR     MOTOR_TT_2
#define ROLLER_DUTY    1.0f
#define LIFT_DUTY      1.0f
#define SWITCH_DEADZONE 0.3f

#define GRIPPER_SERVO     PWM_SERVO_1
#define SERVO_OPEN_DUTY   0.075f
#define SERVO_CLOSE_DUTY  0.025f
#define SERVO_INTO_DUTY   SERVO_OPEN_DUTY

#define CH_STR  1
#define CH_FWD  2
#define CH_ROT  4
#define CH_VrA  8
#define CH_SWB  5
#define CH_VrB  6

pid_t speed_pid;
int target_rpm = 500;
int current;

static inline float clampf(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static inline int sw3_pos(float v)
{
    if (v >  SWITCH_DEADZONE) return +1;
    if (v < -SWITCH_DEADZONE) return -1;
    return 0;
}

static float gripper_duty = SERVO_INTO_DUTY;


static inline void gripper_drive(float dir)
{
    if (dir > 0.0f)
    {
			  gripper_duty = SERVO_OPEN_DUTY;
    }
    else if (dir < 0.0f)
    {
        gripper_duty  = SERVO_CLOSE_DUTY;
    }
    else
    {
        
    }
		pwm_setDuty(GRIPPER_SERVO, gripper_duty );
}

void robot_main(void *args)
{
    sbus_t remote;
    (void)args;

    motor_tt_enable(MOTOR_TT_1);
    motor_tt_enable(MOTOR_TT_4);
    motor_tt_enable(MOTOR_TT_6);
    motor_tt_enable(MOTOR_TT_5);
    motor_tt_enable(ROLLER_MOTOR);
    motor_tt_enable(LIFT_MOTOR);

   
    gripper_drive(0.0f);

    const pid_config_t pid_config =
    {
        .kp = 2.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .output_min = -500.0f,
        .output_max = 500.0f,
        .integral_min = -250.0f,
        .integral_max = 250.0f
    };
    pid_init(&speed_pid, &pid_config);

    uint32_t last_wake = os_getTime();

    for (;;)
    {
        if (sbus_get(&remote) == 0 && remote.valid && remote.online)
        {
            float str =  remote.channel[CH_STR];
            float fwd =  remote.channel[CH_FWD];
            float rot = -remote.channel[CH_ROT];

            if (fabsf(fwd) < stick_deadzone) fwd = 0.0f;
            if (fabsf(str) < stick_deadzone ) str = 0.0f;
						if (fabsf(rot) < stick_deadzone ) rot = 0.0f;

            float lf = SIGN_LF * (fwd + str - rot);
            float rf = SIGN_RF * (fwd - str + rot);
            float lb = SIGN_LB * (fwd - str - rot);
            float rb = SIGN_RB * (fwd + str + rot);

            motor_tt_setDuty(MOTOR_TT_4, clampf(lf, -duty_limit, duty_limit));
            motor_tt_setDuty(MOTOR_TT_1, clampf(rf, -duty_limit, duty_limit));
            motor_tt_setDuty(MOTOR_TT_5, clampf(lb, -duty_limit, duty_limit));
            motor_tt_setDuty(MOTOR_TT_6, clampf(rb, -duty_limit, duty_limit));

            int vra = sw3_pos(remote.channel[CH_VrA]);
            int swb = sw3_pos(remote.channel[CH_SWB]);
            int vrb = sw3_pos(remote.channel[CH_VrB]);   

            float roller = 0.0f;
            if (vra > 0.5)      roller = -ROLLER_DUTY;
            else if (vra < -0.5) roller =  ROLLER_DUTY;
            motor_tt_setDuty(ROLLER_MOTOR, roller);

            float lift = 0.0f;
            if (swb > 0)      lift =  -LIFT_DUTY;
            else if (swb < 0) lift = LIFT_DUTY;
            motor_tt_setDuty(LIFT_MOTOR, lift);

            if (vrb > 0.5)      gripper_drive(+1.0f);  /* ???? */
            else if (vrb < -0.5) gripper_drive(-1.0f);  /* ???? */
            else              gripper_drive(0.0f);   /* ?? */
        }
        else
        {
            motor_tt_setDuty(MOTOR_TT_4, 0.0f);
            motor_tt_setDuty(MOTOR_TT_1, 0.0f);
            motor_tt_setDuty(MOTOR_TT_5, 0.0f);
            motor_tt_setDuty(MOTOR_TT_6, 0.0f);
            motor_tt_setDuty(ROLLER_MOTOR, 0.0f);
            motor_tt_setDuty(LIFT_MOTOR, 0.0f);
            gripper_drive(0.0f);
        }

        os_delayUntil(&last_wake, ctrl_period_ms);
    }
}