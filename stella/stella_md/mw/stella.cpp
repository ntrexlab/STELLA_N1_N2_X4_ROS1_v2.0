#include "MW_value.hpp"
#include "MW_serial.hpp"

struct Motor_Configuration_value stella;
struct Differential_Mobile_Robot_value Differential_MobileRobot_SetValue;

int stella_setting(int select);
int Differential_MobileRobot_Setting();
int stella_setting_check(int channel);
int Differential_MobileRobot_setting_check();

int stella_setting_check(int channel)
{
    if (stella.encoder_ppr[channel] != MyMotorConfiguration.encoder_ppr[channel])
        return 0;
    if (stella.Max_voltage[channel] != MyMotorConfiguration.Max_voltage[channel])
        return 0;
    if (stella.Max_current[channel] != MyMotorConfiguration.Max_current[channel])
        return 0;

    if (stella.Max_velocity[channel] != MyMotorConfiguration.Max_velocity[channel])
        return 0;
    if (stella.Max_velocity[channel] != MyMotorConfiguration.Max_velocity[channel])
        return 0;
    if (stella.Acceleration[channel] != MyMotorConfiguration.Acceleration[channel])
        return 0;
    if (stella.Deceleration[channel] != MyMotorConfiguration.Deceleration[channel])
        return 0;

    if (stella.overvoltage_limit[channel] != MyMotorConfiguration.overvoltage_limit[channel])
        return 0;
    if (stella.undervoltage_limit[channel] != MyMotorConfiguration.undervoltage_limit[channel])
        return 0;
    if (stella.overcurrent_limit[channel] != MyMotorConfiguration.overcurrent_limit[channel])
        return 0;

    if (stella.direction[channel] != MyMotorConfiguration.direction[channel])
        return 0;

    if (stella.velocity_p_gain[channel] != MyMotorConfiguration.velocity_p_gain[channel])
        return 0;
    if (stella.velocity_i_gain[channel] != MyMotorConfiguration.velocity_i_gain[channel])
        return 0;

    return 1;
}


int Differential_MobileRobot_setting_check()
{
    if (Differential_MobileRobot_SetValue.axle_length != Differential_MobileRobot.axle_length)
        return 0;
    if (Differential_MobileRobot_SetValue.wheel_radius != Differential_MobileRobot.wheel_radius)
        return 0;
    if (Differential_MobileRobot_SetValue.gear_ratio != Differential_MobileRobot.gear_ratio)
        return 0;

    return 1;
}
int stella_setting(int select)
{
    switch (select)
    {
    case 0:
        // 모터사양 참고
        stella.encoder_ppr[channel_1] = 2000;
        stella.encoder_ppr[channel_2] = 2000;

        stella.Max_voltage[channel_1] = 20.0;
        stella.Max_voltage[channel_2] = 20.0;

        stella.Max_current[channel_1] = 2.3;
        stella.Max_current[channel_2] = 2.3;

        stella.Max_velocity[channel_1] = 3000;
        stella.Max_velocity[channel_2] = 3000;

        stella.Acceleration[channel_1] = 3000;
        stella.Acceleration[channel_2] = 3000;

        stella.Deceleration[channel_1] = 3000;
        stella.Deceleration[channel_2] = 3000;

        // 제어기 안전
        stella.overvoltage_limit[channel_1] = 30.0;
        stella.overvoltage_limit[channel_2] = 30.0;

        stella.undervoltage_limit[channel_1] = 9.0;
        stella.undervoltage_limit[channel_2] = 9.0;

        stella.overcurrent_limit[channel_1] = 3.0;
        stella.overcurrent_limit[channel_2] = 3.0;

        // 하드웨어 참고
        stella.direction[channel_1] = Reverse_Direction;
        stella.direction[channel_2] = Not_Change;

        stella.velocity_p_gain[channel_1] = 3.3;
        stella.velocity_p_gain[channel_2] = 3.3;

        stella.velocity_i_gain[channel_1] = 0.06;
        stella.velocity_i_gain[channel_2] = 0.06;

        // 로봇 기구학 셋팅
        Differential_MobileRobot_SetValue.axle_length   = 0.33;            // 바퀴간 거리             (m)        
        Differential_MobileRobot_SetValue.wheel_radius  = 0.0875;           // 바퀴 반지름             (m)        
        Differential_MobileRobot_SetValue.gear_ratio    = 27;              // 모터회전수 / 바퀴회전수   (감속비율) 

        // 적용
        Set_MotorConfiguration(channel_1, &stella);
        Set_MotorConfiguration(channel_2, &stella);

        Set_Differential_MobileRobot_value(&Differential_MobileRobot_SetValue);
 
    }

    return 1;
}

int Robot_Fault_Checking_RESET()
{
    Fault();
    
    if(MyMotorControl.Fault[Motor_select::channel_1] != NO_FAULT)
    {
        printf("\n[MotorDriver INFO] Motor channel_1 fault detected \r\n");
    }
    
    if(MyMotorControl.Fault[Motor_select::channel_2] != NO_FAULT)
    {
        printf("\n[MotorDriver INFO] Motor channel_2 fault detected \r\n");
    }

    if((MyMotorControl.Fault[Motor_select::channel_1] != NO_FAULT) || (MyMotorControl.Fault[Motor_select::channel_2] != NO_FAULT))
    {
        printf("\n[MotorDriver INFO] MotorDriver RESET MODE\r\n\n");
        
        FunctionMotorControlSystem(RESET_CONTROLLE);
        return 1;
    }
}

int Robot_Setting(int choice)
{
    stella_setting(choice);

    int res = stella_setting_check(channel_1) && stella_setting_check(channel_2) && Differential_MobileRobot_setting_check();

    if (!res)
    {
        printf("\n[MotorDriver INFO] Robot Setting .... \n\n ");

        FunctionMotorControlSystem(SAVE_CONFIG_FLASH);

        Get_Differential_MobileRobot_value();

        Get_MotorConfiguration(channel_1);
        Get_MotorConfiguration(channel_2);

        InformationsPrintf();

        return 1;
    }
    else
    {
      printf("\n[MotorDriver INFO] Stella Robot Setting OK\n\n ");
    }
    

    return 1;
}
