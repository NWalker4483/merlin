#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
#include "config.h"

#define SPR 200
#define PPR 1000

BLA::Matrix<6, 6> motor_reductions = {
    1. / 48., 0, 0, 0, 0, 0,
    0, 1. / 48., 0, 0, 0, 0,
    0, -1. / 48., 1. / 48., 0, 0, 0,
    0, 0, 0, 1. / 24., 0, 0,
    0, 0, 0, -1. / 28.8, 1. / 28.8, 0,
    0, 0, 0, -1. / 12., 1. / 24., 1. / 24.};

BLA::Matrix<6, 6> radians_per_step;
BLA::Matrix<6, 6> radians_per_step_inv;

BLA::Matrix<6, 1> target_speeds;
BLA::Matrix<6, 1> target_poses;

BLA::Matrix<6, 1> current_poses;
BLA::Matrix<6, 1> min_joint_travel;

void setup_steppers()
{
    stepper[0] = &stepper1;
    stepper[1] = &stepper2;
    stepper[2] = &stepper3;
    stepper[3] = &stepper4;
    stepper[4] = &stepper5;
    stepper[5] = &stepper6;
    for (int i = 0; i < 6; i++)
        stepper[i]->setAcceleration(2000);
}

void update_arm_state()
{
    BLA::Matrix<6, 1> steps_taken;
    for (int i = 0; i < 6; i++) steps_taken(i) = stepper[i]->currentPosition();
    current_poses = radians_per_step * steps_taken;
}

void solve_motor_speeds()
{
    BLA::Matrix<6, 1> delta_theta;
    // Compute Errors
    update_arm_state();
    for (int joint = 0; joint < 6; joint++)
    {
        delta_theta(joint) = target_poses(joint) - current_poses(joint);
    }

    for (int stage = 0; stage < 6; stage++)
    {
        float min_travel_time = 10000;
        for (int joint = 0; joint < 6; joint++)
        {
            if (abs(delta_theta(joint)) >= radians_per_step(joint, joint) and target_speeds(joint) != 0)
            {
                float travel_time = delta_theta(joint) / target_speeds(joint);
                if (travel_time < min_travel_time)
                    min_travel_time = travel_time;
            }
            if (min_travel_time == 10000)
                break;
        }
        BLA::Matrix<6, 1> steps_per_second = radians_per_step_inv * target_speeds;
        BLA::Matrix<6, 1> steps_to_take = steps_per_second * min_travel_time;
        BLA::Matrix<6, 1> sub_delta_theta = radians_per_step * steps_to_take;
        delta_theta -= sub_delta_theta;
        for (int i = 0; i < 6; i++)
            cmd_buffer[stage][0][i] = steps_to_take(i);
        for (int i = 0; i < 6; i++)
            cmd_buffer[stage][1][i] = steps_per_second(i);
        cmd_len = stage + 1;
    }
    cmd_idx = -1;
}

void handle_commands()
{
    if (Serial.available() > 0)
    {
        int cmd = Serial.read();
        open_float temp;
        if (cmd == 'R')
        {
            update_arm_state();
            open_float temp;
            for (int joint = 0; joint < 6; joint++)
            {
                temp.value = current_poses(joint);
                Serial.write(temp.bytes, 4);
            }
        }

        if (cmd == 'W')
        {
            solve_motor_speeds();
            restart = true; 
        }
    }
}

void setup()
{
    setup_steppers();
    // load_saved_state();
    // Pre Compute Useful Variables
    radians_per_step = motor_reductions * (6.283 / SPR);
    auto radians_per_step_inv = radians_per_step;
    Invert(radians_per_step_inv);
}

void run_commands()
{
    bool done = true;
    for (int i = 0; i < 6; i++)
        done = done && (not stepper[i]->isRunning());

    if (done || restart)
    {
        restart = false;
        if (cmd_idx == (cmd_len - 1))
            return;
        else
            cmd_idx += 1;
        for (int i = 0; i < 6; i++)
        {
            stepper[i]->setMaxSpeed(cmd_buffer[cmd_idx][1][i]);
            stepper[i]->setSpeed(cmd_buffer[cmd_idx][1][i]);
            stepper[i]->move(cmd_buffer[cmd_idx][0][i]);
        }
    }
    for (int i = 0; i < 6; i++) stepper[i]->runSpeed();
}

void loop()
{
    handle_commands();
    run_commands();
}
