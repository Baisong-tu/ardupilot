#include "Copter.h"

/*
 * Init and run calls for tudao flight mode
 */
// guided_init - initialise guided controller
bool ModeTudao::init(bool ignore_checks)
{
    const AP_InertialSensor &ins = AP::ins();
    running_num_int = 0;
    pid_float = 0;
    Gyro_latest = ahrs.get_gyro_latest();//vector of rotational rates in radians/sec
    Acc_ef_mss = ahrs.get_accel_ef_blended();//// accelerometer values in the earth frame in m/s/s
    Gyro_latest = ins.get_gyro(0);
    Acc_ef_mss = ins.get_accel(0);

    gcs().send_text(MAV_SEVERITY_CRITICAL, "Tudao: Missing Start");

    //get eluer roll pitch yaw and dispaly it in the computer
    Roll = 57.3 * ahrs.get_roll();
    Pitch = 57.3 *  ahrs.get_pitch();
    Yaw = 57.3 * ahrs.get_yaw();
    Gyro_x = Gyro_latest.x;
    Gyro_y = Gyro_latest.y;
    Gyro_z = Gyro_latest.z;
    //加速度计数据不是原始数据，此处需要再修改
    Acc_x = Acc_ef_mss.x;
    Acc_y = Acc_ef_mss.y;
    Acc_z = Acc_ef_mss.z;
    gcs().send_text(MAV_SEVERITY_CRITICAL,
                     "ACCX is %.2f",
                     Acc_x);
    gcs().send_text(MAV_SEVERITY_CRITICAL,
                     "ACCY is %.2f",
                     Acc_y);
    gcs().send_text(MAV_SEVERITY_CRITICAL,
                     "ACCZ is %.2f",
                     Acc_z);
    if(attitude_control->_rate_bf_ff_enabled == 0)
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "FF Disable");
    }
    else
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "FF Enable");
    }

    return true;
}
// tudao_run - runs the main tudao controller
// should be called at 100hz or more
void ModeTudao::run()
{
    //inceremit the times
    running_num_int++;
    // convert pilot input to lean angles
    float target_roll, target_pitch;//unit is centidegree
    //float target_yaw_rate;//unit is centidegree/s
    float target_yaw_rate;
    target_roll = 500;
    target_pitch = 500;
    if(running_num_int <= 2000)
    {
        target_roll = 500;
        target_pitch = 500;
        target_yaw_rate = 0;
        if (running_num_int == 1)
        {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Tudao: Missing 1111");
        }
    }
    else if(running_num_int <= 4000)
    {
        target_roll = -500;
        target_pitch = -500;
        target_yaw_rate = 0;
        if (running_num_int == 2002)
        {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Tudao: Missing 2222");
        }
    }
    else if(4000 < running_num_int)
    {
        get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (running_num_int == 4001)
        {
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Tudao: Missing 3333");
        }
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // output pilot's throttle
    attitude_control->set_throttle_out(get_pilot_desired_throttle(),
                                       true,
                                       g.throttle_filt);
}
