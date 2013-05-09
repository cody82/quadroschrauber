#include "quadroschrauber.h"


Vector3 QuadroSchrauber::forward(1,0,0);
Vector3 QuadroSchrauber::left(0,-1,0);
Vector3 QuadroSchrauber::up(0,0,-1);


#include "MadgwickAHRS.h"



extern "C" void mbed_reset();

// I2C devices(8bit):
// BlinkM: 0x12
// HMC5883: 0x3C
// MPU6050: 0xD0
// MS5611: 0xEE
// GPS: 0x22
    
QuadroSchrauber::QuadroSchrauber()
        : i2c(p28,p27), blinkm(i2c, 0x12), imu(i2c, 0x68, timer), pc(p13,p14, 512, 128), /*pc(USBTX, USBRX, 256, 128), bluetooth(p13,p14, 256, 128),*/ remote_serial(p9, p10, 16, 64),
        pwm1(p21), pwm2(p22), pwm3(p23), pwm4(p24),
        led1(LED1), led2(LED2), led3(LED3), led4(LED4), ok(false),
        motor_left(pwm1), motor_front(pwm2), motor_back(pwm3), motor_right(pwm4),
        counter(0), mavlink_system_mode(MAV_MODE_PREFLIGHT),
        local("local"), save_flag(false), remote_time(100),
        gps(i2c), battery_voltage(p20), voltage(10), loop_time_max(0), voltage_ok(true),
        ultrasound(p17, p18)
{
    
    mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
    mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
    mavlink_system.type = MAV_TYPE_QUADROTOR;   ///< This system is an airplane / fixed wing

    i2c.frequency(100000);

    pc.baud(115200);
    //bluetooth.baud(115200);
    remote_serial.baud(115200);
    //pc.attach<QuadroSchrauber>(this,&QuadroSchrauber::ReceiveCallback, MODSERIAL::RxIrq);
    mavlink_send_text("Hello QuadroSchrauber");

    led1 = 1;
    pwm1.period_ms(10);
    pwm2.period_ms(10);
    pwm3.period_ms(10);
    pwm4.period_ms(10);

    pwm1.pulsewidth_us(1000);
    pwm2.pulsewidth_us(1000);
    pwm3.pulsewidth_us(1000);
    pwm4.pulsewidth_us(1000);

    led2 = 1;

    blinkm.StopScript();
    led3 = 1;
    blinkm.Fade(0xFF, 0xFF, 0x00);

    if (imu.Init()) {
        led4 = 1;
        blinkm.Fade(0x00, 0xFF, 0x00);
        ok = true;
    }
    
    char tmp[32];
    for(int i=1;i<7;++i)
    {
        sprintf(tmp, "coeff: %d", imu.ms5611.GetCoefficient(i));
        mavlink_send_text(tmp);
    }
    sprintf(tmp, "pressure: %f", imu.ms5611.GetPressure());
    mavlink_send_text(tmp);
    sprintf(tmp, "temperature: %f", imu.ms5611.GetTemperature());
    mavlink_send_text(tmp);
    sprintf(tmp, "altitude: %f", imu.ms5611.GetAltitude());
    mavlink_send_text(tmp);
    
    for (int address=0; address<256; address+=2) {
        if (!i2c.write(address, NULL, 0)) { // 0 returned is ok
            sprintf(tmp, "device found at address 0x%02X\n", address);
            mavlink_send_text(tmp);
        }
    } 
    
    LoadConfig();
}

void QuadroSchrauber::Reset()
{
    mbed_reset();
}

void QuadroSchrauber::LoadConfigValue(const char *name, float value)
{
    ConfigValue v[] = QUADROSCHRAUBER_CONFIG;
    
    for(int i=0;i<QUADROSCHRAUBER_CONFIG_COUNT;++i)
    {
        if(strcmp(name, v[i].name) == 0)
        {
            *v[i].value = value;
            break;
        }
    }
}

void QuadroSchrauber::SaveConfig()
{
    FILE *fp = fopen("/local/qs_cfg.txt", "w");
    if(!fp)
    {
        return;
    }
    
    ConfigValue v[] = QUADROSCHRAUBER_CONFIG;
    for(int i=0;i<QUADROSCHRAUBER_CONFIG_COUNT;++i)
    {
        fprintf(fp, "%s %f\r\n", v[i].name, *v[i].value);
    }
    
    fclose(fp);
}

void QuadroSchrauber::LoadConfig()
{
    FILE *fp = fopen("/local/qs_cfg.txt", "r");
    if(!fp)
    {
        return;
    }
    
    char name[24];
    float value;
    while(fgets((char*)buf, 32, fp))
    {
        if(sscanf((const char*)buf, "%s %f", name, &value) == 2)
        {
            LoadConfigValue(name, value);
        }
    }
    
    fclose(fp);
}
               
void QuadroSchrauber::ReceiveChar(uint8_t c, uint8_t com)
{
    mavlink_message_t msg;
    mavlink_status_t status;

    if (mavlink_parse_char(com, c, &msg, &status))
    {
        switch (msg.msgid)
        {
            case MAVLINK_MSG_ID_HEARTBEAT:
            {
                // E.g. read GCS heartbeat and go into
                // comm lost mode if timer times out
                led3 = !led3;
            }
            break;
            case MAVLINK_MSG_ID_SET_QUAD_MOTORS_SETPOINT:
            {
                uint16_t tmp;

                tmp = mavlink_msg_set_quad_motors_setpoint_get_motor_front_nw(&msg);
                motor_front.SetThrottleMillis(tmp);

                tmp = mavlink_msg_set_quad_motors_setpoint_get_motor_right_ne(&msg);
                motor_right.SetThrottleMillis(tmp);

                tmp = mavlink_msg_set_quad_motors_setpoint_get_motor_back_se(&msg);
                motor_back.SetThrottleMillis(tmp);

                tmp = mavlink_msg_set_quad_motors_setpoint_get_motor_left_sw(&msg);
                motor_left.SetThrottleMillis(tmp);

                remote.active = false;
                led3 = !led3;
                break;
            }
            case MAVLINK_MSG_ID_MANUAL_CONTROL:
            {
                float roll = mavlink_msg_manual_control_get_roll(&msg);/* 0...1 */
                float pitch = mavlink_msg_manual_control_get_pitch(&msg);/* 0...1 */
                float yaw = mavlink_msg_manual_control_get_yaw(&msg);/* 0...1 */
                float throttle = mavlink_msg_manual_control_get_thrust(&msg); /* 0...1 */
                
                throttle -= 0.5f;
                if(throttle < 0.0f)
                    throttle = 0.0f;
                    
                remote.roll = -pitch;
                remote.pitch = -roll;
                remote.yaw = -yaw;
                remote.throttle = throttle;
                remote.active = true;
                
                led3 = !led3;
                break;
            }
            case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
            {
                mavlink_message_t msg2;
                uint16_t len;
                
                mavlink_send_text("REQUEST");
                
                ConfigValue v[] = QUADROSCHRAUBER_CONFIG;
    
                for(int i=0;i<QUADROSCHRAUBER_CONFIG_COUNT;++i)
                {
                    mavlink_msg_param_value_pack(mavlink_system.sysid, mavlink_system.compid, &msg2,
                        v[i].name, *v[i].value, MAVLINK_TYPE_FLOAT, QUADROSCHRAUBER_CONFIG_COUNT, i);
                    len = mavlink_msg_to_send_buffer(buf, &msg2);
                    Send(buf, len);
                }
        
                led3 = !led3;
                break;
            }
            case MAVLINK_MSG_ID_PARAM_SET:
            {
                char id[16];
                uint16_t len = mavlink_msg_param_set_get_param_id(&msg, id);
                float v = mavlink_msg_param_set_get_param_value(&msg);
                
                int index = -1;
                ConfigValue cv[] = QUADROSCHRAUBER_CONFIG;
    
                for(int i=0;i<QUADROSCHRAUBER_CONFIG_COUNT;++i)
                {
                    if(strncmp(cv[i].name, id, len) == 0)
                    {
                        *cv[i].value = v;
                        index = i;
                        break;
                    }
                }
                
                if(index >= 0)
                {
                    save_flag = true;
                
                    mavlink_message_t msg2;
                    
                    mavlink_msg_param_value_pack(mavlink_system.sysid, mavlink_system.compid, &msg2,
                        id, v, MAVLINK_TYPE_FLOAT, QUADROSCHRAUBER_CONFIG_COUNT, index);
                        
                    len = mavlink_msg_to_send_buffer(buf, &msg2);
    
                    Send(buf, len);
                    
                    led3 = !led3;
                }
                break;
            }
            case MAVLINK_MSG_ID_SET_MODE:
            {
                uint8_t mode = mavlink_msg_set_mode_get_base_mode(&msg);
                if(mode == MAV_MODE_MANUAL_DISARMED)
                    mode = MAV_MODE_MANUAL_ARMED;
                
                imu.Calibrate();
                    
                mavlink_system_mode = mode;
                led3 = !led3;
                break;
            }
            case MAVLINK_MSG_ID_COMMAND_LONG:
            {
                uint16_t cmd = mavlink_msg_command_long_get_command(&msg);
                switch(cmd)
                {
                    case MAV_CMD_PREFLIGHT_CALIBRATION:
                        led3 = !led3;
                        imu.Calibrate();
                    break;
                }
                break;
            }
            default:
                break;
        }
    }
}

void QuadroSchrauber::ReceiveCallback()
{
}

void QuadroSchrauber::Shutdown()
{
    motor_left.SetThrottleMillis(0);
    motor_right.SetThrottleMillis(0);
    motor_front.SetThrottleMillis(0);
    motor_back.SetThrottleMillis(0);

    mavlink_send_text("QuadroSchrauber shutdown");

    blinkm.Fade(0xFF, 0x00, 0x00);
}

void QuadroSchrauber::mavlink_send_heartbeat()
{
    const uint8_t system_type = MAV_TYPE_QUADROTOR;
    const uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;

    //uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
    //uint8_t system_mode = MAV_MODE_MANUAL_ARMED; ///< Booting up
    uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
    //uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight
    uint8_t system_state = MAV_STATE_ACTIVE; ///< System ready for flight

    mavlink_message_t msg;
    //uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, system_type, autopilot_type, mavlink_system_mode, custom_mode, system_state);

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    Send(buf, len);
}


void QuadroSchrauber::mavlink_send_text(const char *text)
{
    mavlink_message_t msg;


    mavlink_msg_statustext_pack(mavlink_system.sysid, mavlink_system.compid, &msg, 0, text);

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(buf, len);
}


void QuadroSchrauber::mavlink_send_data()
{
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];


    mavlink_msg_scaled_imu_pack(mavlink_system.sysid, mavlink_system.compid, &msg, (uint32_t)timer.read_ms(),
                                (int16_t)(controller.accel_filter.x * 1000.0f), (int16_t)(controller.accel_filter.y * 1000.0f), (int16_t)(controller.accel_filter.z * 1000.0f),
                                (int16_t)(controller.gyro_filter.x * 1000.0f), (int16_t)(controller.gyro_filter.y * 1000.0f), (int16_t)(controller.gyro_filter.z * 1000.0f),
                                (int16_t)(controller.magneto_filter.x * 1000.0f),(int16_t)(controller.magneto_filter.y * 1000.0f),(int16_t)(controller.magneto_filter.z * 1000.0f));

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(buf, len);
    
    /*mavlink_msg_attitude_pack(mavlink_system.sysid, mavlink_system.compid, &msg, (uint32_t)timer.read_ms(),
        controller.combined.x,controller.combined.y,controller.combined.z,sensors.gyro.x,sensors.gyro.y,sensors.gyro.z);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(buf, len);
    */
    
    mavlink_msg_attitude_pack(mavlink_system.sysid, mavlink_system.compid, &msg, (uint32_t)timer.read_ms(),
        controller.ahrs.x,controller.ahrs.y,controller.ahrs.z,controller.ahrs_dev.x,controller.ahrs_dev.y,controller.ahrs_dev.z);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(buf, len);
    
    
    mavlink_msg_attitude_quaternion_pack(mavlink_system.sysid, mavlink_system.compid, &msg, (uint32_t)timer.read_ms(),
        //q0, q1, q2, q3, imu.gyro.x, imu.gyro.y, imu.gyro.z);
        //q0, q1, q2, q3, controller.inner_pid_output.x, controller.outer_pid_output.x, loop_time_max);
        //q0, q1, q2, q3, controller.inner.error_integral.x, controller.inner.error_integral.y, loop_time_max);
        q0, q1, q2, q3, remote.switch1 + remote.switch2 * 2, ultrasound.Distance(), loop_time_max);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(buf, len);
      
        
    mavlink_msg_rc_channels_scaled_pack(mavlink_system.sysid, mavlink_system.compid, &msg, (uint32_t)timer.read_ms(),
        0, remote.throttle * 10000, remote.yaw * 10000, remote.pitch * 10000, remote.roll * 10000, ch1[4], ch1[5], 0, 0, 0);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(buf, len);
        
    mavlink_msg_servo_output_raw_pack(mavlink_system.sysid, mavlink_system.compid, &msg, (uint32_t)timer.read_ms() * 1000,
        0, motors.motor_left * 10000, motors.motor_right * 10000, motors.motor_front * 10000, motors.motor_back * 10000, 0, 0, 0, 0);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    Send(buf, len);
    
    if(counter % 40 == 0)
    {
        mavlink_msg_sys_status_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
                               gps.ok, gps.ok, gps.ok, 0/*load*/, (uint16_t)(voltage * 1000.0f), 0/*current*/, 0/*battery_remaining*/, 0, 0, 0, 0, 0, 0);

        len = mavlink_msg_to_send_buffer(buf, &msg);
        Send(buf, len);
    }
       
    if(counter % 40 == 20)
    { 
        mavlink_msg_global_position_int_pack(mavlink_system.sysid, mavlink_system.compid, &msg, (uint32_t)timer.read_ms(),
                               gps.GetLatitude() * 10000000, gps.GetLongitude() * 10000000, imu.ms5611.GetAltitude() * 1000, imu.ms5611.GetAltitude() * 1000,
                               0,0,0,65535);
        len = mavlink_msg_to_send_buffer(buf, &msg);
        Send(buf, len);
    }
}

void QuadroSchrauber::Send(uint8_t c)
{
    //bluetooth.putc(c);
    pc.putc(c);
}

void QuadroSchrauber::Send(uint8_t *buf, int len)
{
    for (int i=0; i<len; ++i)
        Send(buf[i]);
}

void QuadroSchrauber::ReceiveRemote(char c)
{
    led2 = !led2;
}

float QuadroSchrauber::GetBatteryVoltage()
{
    return battery_voltage * 3.3f * 6.666f;
}

#define RC_THROTTLE_CHANNEL (0)
#define RC_PITCH_CHANNEL (2)
#define RC_ROLL_CHANNEL (3)
#define RC_YAW_CHANNEL (1)
#define RC_SWITCH1_CHANNEL (4)
#define RC_SWITCH2_CHANNEL (5)
#define RC_MAX (1710)
#define RC_MIN (330)
#define RC_MID ((RC_MAX + RC_MIN) / 2)
#define RC_THROTTLE_INVERT (1)
#define RC_PITCH_INVERT (-1)
#define RC_ROLL_INVERT (-1)
#define RC_YAW_INVERT (-1)

void QuadroSchrauber::Update(float dtime)
{
    int time = timer.read_us();
    
    if(save_flag && (counter % 100 == 0))
    {
        SaveConfig();
        save_flag = false;
    }
    
    voltage = GetBatteryVoltage();
    if(voltage < BATTERY_MIN_VOLTAGE)
    {
        if(voltage_ok)
        {
            voltage_ok = false;
            blinkm.Fade(255,255,0);
            mavlink_send_text("LOW BATTERY");
        }
    }
    else
    {
        if(!voltage_ok)
        {
            voltage_ok = true;
            mavlink_send_text("Battery OK");
            if(remote.active)
                blinkm.Fade(0,0,255);
            else
                blinkm.Fade(0,255,0);
        }
    }
    
    //remote_serial.putc(5);
    
        while (pc.readable())
            ReceiveChar(pc.getc(), MAVLINK_COMM_0);
        //while (bluetooth.readable())
        //    ReceiveChar(bluetooth.getc(),MAVLINK_COMM_1);
            
        if(remote_time < 100)
            remote_time++;
        if(remote_time == 100)
        {
            if(voltage_ok)
                blinkm.Fade(0,255,0);
            remote.active = false;
        }
        
        /*while (remote_serial.readable())
        {
            if(remote_time == 100)
            {
                blinkm.Fade(0,0,255);
            }
            remote_time = 0;
            ReceiveRemote(remote_serial.getc());
        }*/
        int rx = remote_serial.rxBufferGetCount();
        if(rx == 16)
        {
            for(int i=0;i<16;++i)
            {
                remote_data[i] = remote_serial.getc();
            }
            led2 = !led2;
            
            
            for(int i=1;i<8;++i)
            {
                uint16_t tmp = (remote_data[i*2] * 256 + remote_data[i*2+1]);
                uint16_t channel = (tmp >> 11) & 7;
                if(channel < 6)
                    ch1[channel] = tmp & ((1<<11)-1);
            }
            remote.pitch = (((float)ch1[RC_PITCH_CHANNEL]) - RC_MID) / (RC_MAX - RC_MID) * RC_PITCH_INVERT;
            remote.roll = (((float)ch1[RC_ROLL_CHANNEL]) - RC_MID) / (RC_MAX - RC_MID) * RC_ROLL_INVERT;
            remote.yaw = (((float)ch1[RC_YAW_CHANNEL]) - RC_MID) / (RC_MAX - RC_MID) * RC_YAW_INVERT;
            remote.throttle = (((float)ch1[RC_THROTTLE_CHANNEL]) - RC_MIN) / (RC_MAX - RC_MIN) * RC_THROTTLE_INVERT;
            remote.switch1 = ch1[RC_SWITCH1_CHANNEL] > RC_MID;
            remote.switch2 = ch1[RC_SWITCH2_CHANNEL] > RC_MID;
            
            if(remote_time == 100)
            {
                if(voltage_ok)
                    blinkm.Fade(0,0,255);
                remote.active = true;
            }
            remote_time = 0;
            
        }
        else if(rx > 16)
        {
            while(remote_serial.readable())
                remote_serial.getc();
        }
        
        if(!remote.active)
        {
            remote.roll = remote.pitch = remote.yaw = remote.throttle = 0.0f;
        }
        
        
        if (counter % 40 == 0)
            ultrasound.Trigger();
            
           //gps.Update();
        /*SensorInput tmp;
        SensorInput tmp2;
        SensorInput tmp3;
        const int wait_time_us = 250;
        if (!(ok = GetSensorData(dtime, tmp)))
        {
            mavlink_send_text("SENSOR FAIL");
            return;
        }
        //wait_us(wait_time_us);
        if (!(ok = GetSensorData(dtime, tmp2)))
        {
            mavlink_send_text("SENSOR FAIL");
            return;
        }
        //wait_us(wait_time_us);
        */
        if (!(ok = GetSensorData(dtime, sensors)))
        {
            mavlink_send_text("SENSOR FAIL");
            return;
        }
        //wait_us(wait_time_us);
        /*
        if (!(ok = GetSensorData(dtime, tmp3)))
        {
            mavlink_send_text("SENSOR FAIL");
            return;
        }
        
        sensors.accel = sensors.accel * 0.25f + tmp.accel * 0.25f + tmp2.accel * 0.25f + tmp3.accel * 0.25f;
        sensors.gyro = sensors.gyro * 0.25f + tmp.gyro * 0.25f + tmp2.gyro * 0.25f + tmp3.gyro * 0.25f;
        sensors.magneto = sensors.magneto * 0.25f + tmp.magneto * 0.25f + tmp2.magneto * 0.25f + tmp3.magneto * 0.25f;
        */
        if (!(ok = GetRemoteData(remote)))
        {
            mavlink_send_text("REMOTE FAIL");
            return;
        }

        controller.Update(dtime, remote, sensors, motors);
        
        if(!(ok = SetMotors(motors)))
        {
            mavlink_send_text("MOTOR FAIL");
            return;
        }


        if (counter % 4 == 0)
        {
            led1 = !led1;
            mavlink_send_data();
        }

        if (counter % 100 == 0)
        {
            mavlink_send_heartbeat();
        }
        
        if (counter == 100)
            imu.Calibrate();
            
        counter = (counter+1) % 100000000;
        
    time = timer.read_us() - time;
    if(time > 0)
        loop_time_max = loop_time_max * 0.5f + time * 0.5f;
}

bool QuadroSchrauber::GetSensorData(float dtime, SensorInput &data)
{
    if (imu.Update(dtime))
    {
        data.accel = imu.accel;
        data.gyro = imu.gyro;
        data.magneto = imu.magneto;
        return true;
    }
    else
        return false;
}

bool QuadroSchrauber::GetRemoteData(RemoteInput &data)
{
    return true;
}

bool QuadroSchrauber::SetMotors(MotorOutput &data)
{
    motor_left.SetThrottle(data.motor_left);
    motor_right.SetThrottle(data.motor_right);
    motor_back.SetThrottle(data.motor_back);
    motor_front.SetThrottle(data.motor_front);
    return true;
}

