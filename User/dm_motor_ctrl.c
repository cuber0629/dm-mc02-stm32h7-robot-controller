#include "dm_motor_drv.h"
#include "dm_motor_ctrl.h"
#include "string.h"
#include "stdbool.h"
#include "math.h"  // 引入数学库（角度转弧度需用到PI）

//#define DEBUG_LOG

#ifdef DEBUG_LOG
#include <stdio.h>    // for sscanf & sprintf
#include "usart.h"
#endif

motor_t motor[num];

#define PI        3.1415926535f  // 圆周率定义
#define P_MAX_RAD 12.5f          // 电机位置最大限制（rad），对应原tmp.PMAX

// 定义每个电机的软限位
static const float motor_min_pos[num] = {
    -0.5f,				// Left CAN ID 1
    -0.1f,				// CAN ID 2
    -1.6f,				// CAN ID 3
    -0.5f,				// CAN ID 4
    -1.6f,				// CAN ID 5
    -0.2f,				// CAN ID 6
    -0.75f,				// CAN ID 7
    -0.5f,				// CAN ID 8
    0.0f,					// CAN ID 9
    -1.8f,				// Right CAN ID 1
    -0.5f,				// CAN ID 2
    -1.6f,				// CAN ID 3
    -1.0f,				// CAN ID 4
    -1.6f,				// CAN ID 5
    0.0f,					// CAN ID 6
    -2.3f,				// CAN ID 7
    -0.5f				// CAN ID 8
};

static const float motor_max_pos[num] = {
    1.8f,				// Left CAN ID 1
    0.5,  			// CAN ID 2
    1.0f,				// CAN ID 3
    1.0f,				// CAN ID 4
    1.5f,				// CAN ID 5
    0.4f,				// CAN ID 6
    2.3f,				// CAN ID 7
    0.3f,				// 手
    0.1f,				// 腰
    1.6f,				// Right CAN ID 1
    0.1f, 			// CAN ID 2
    1.0f,				// CAN ID 3
    0.4f,				// CAN ID 4
    1.6f,				// CAN ID 5
    0.8f,				// CAN ID 6
    0.6f,				// CAN ID 7
    0.0f				// CAN ID 8
};

/**
************************************************************************
* @brief:      	dm4310_motor_init: DM4310电机初始化函数
* @param:      	void
* @retval:     	void
* @details:    	初始化1个DM4310型号的电机，设置默认参数和控制模式。
*               设置ID、控制模式和命令模式等信息。
************************************************************************
**/
uint8_t dm_motor_set_pos_by_deg(uint8_t motor_num, float target_deg)
{
    // 1. 校验电机编号合法性
    if(motor_num > RMotor8)
    {
        return 1;  // 电机编号错误
    }

    // 2. 角度转弧度：rad = deg * π / 180
    float target_rad = target_deg * PI / 180.0f;
		if (motor_num == RMotor1 || motor_num == RMotor2 || motor_num == RMotor3 || motor_num == RMotor4  || motor_num == RMotor6 || motor_num == RMotor7)
		{
			target_rad = -target_rad;
		}
    // 3. 限幅：防止超过电机最大位置限制（±12.5rad）
		#ifdef DEBUG_LOG
		char feedback[128];	// 打印debug信息到串口
		sprintf(feedback, "Motor %d set to %f, limit: %f ~ %f\n", motor_num, target_rad, motor_min_pos[motor_num], motor_max_pos[motor_num]);
		HAL_UART_Transmit(&huart1, (uint8_t*)feedback, strlen(feedback), 10);
		#endif
		
    if (target_rad > motor_max_pos[motor_num])
    {
        target_rad = motor_max_pos[motor_num];
    }
    else if (target_rad < motor_min_pos[motor_num])
    {
        target_rad = motor_min_pos[motor_num];
    }

    // 4. 赋值给电机的位置设定值
    motor[motor_num].ctrl.pos_set = target_rad;
		if (motor_num <= LMotor9) {		
			dm_motor_ctrl_send(&hfdcan2, &motor[motor_num]);   // 左手+腰
		}else {
			dm_motor_ctrl_send(&hfdcan1, &motor[motor_num]);   // 右手
    }
		return 0;  // 成功
}

void dm_motor_init(void)
{
	// 初始化Motor1-Motor6的电机结构
	memset(&motor[LMotor1], 0, sizeof(motor[LMotor1]));
	memset(&motor[LMotor2], 0, sizeof(motor[LMotor2]));
	memset(&motor[LMotor3], 0, sizeof(motor[LMotor3]));
	memset(&motor[LMotor4], 0, sizeof(motor[LMotor4]));
	memset(&motor[LMotor5], 0, sizeof(motor[LMotor5]));
	memset(&motor[LMotor6], 0, sizeof(motor[LMotor6]));
	memset(&motor[LMotor7], 0, sizeof(motor[LMotor7]));
	memset(&motor[LMotor8], 0, sizeof(motor[LMotor8]));
	
	memset(&motor[LMotor9], 0, sizeof(motor[LMotor9]));
	
	memset(&motor[RMotor1], 0, sizeof(motor[RMotor1]));
	memset(&motor[RMotor2], 0, sizeof(motor[RMotor2]));
	memset(&motor[RMotor3], 0, sizeof(motor[RMotor3]));
	memset(&motor[RMotor4], 0, sizeof(motor[RMotor4]));
	memset(&motor[RMotor5], 0, sizeof(motor[RMotor5]));
	memset(&motor[RMotor6], 0, sizeof(motor[RMotor6]));
	memset(&motor[RMotor7], 0, sizeof(motor[RMotor7]));
	memset(&motor[RMotor8], 0, sizeof(motor[RMotor8]));

	//第一版调试电机1速度为0.18（未调pid）情况良好
	
	// 设置Motor1的电机信息
	motor[LMotor1].id = 0x01;
	motor[LMotor1].mst_id = 0x00;	
	motor[LMotor1].tmp.read_flag = 1;
	motor[LMotor1].ctrl.mode 	= pos_mode;  // 改为位置模式
	motor[LMotor1].ctrl.vel_set 	= 0.18f;      // 位置模式需要给运行速度
	motor[LMotor1].ctrl.pos_set 	= 0.0f;      
	motor[LMotor1].ctrl.tor_set 	= 0.0f;      // 位置模式下通常扭矩设为0
	motor[LMotor1].ctrl.cur_set 	= 0.0f;      
	motor[LMotor1].ctrl.kp_set 	= 0.0f;      
	motor[LMotor1].ctrl.kd_set 	= 0.0f;      
	motor[LMotor1].tmp.PMAX		= 12.5f; 
	motor[LMotor1].tmp.VMAX		= 30.0f; 
	motor[LMotor1].tmp.TMAX		= 10.0f; 
	
	// 设置Motor2的电机信息
	motor[LMotor2].id = 0x02; 
	motor[LMotor2].mst_id = 0x00;	
	motor[LMotor2].tmp.read_flag = 1;
	motor[LMotor2].ctrl.mode 	= pos_mode;  // 改为位置模式
	motor[LMotor2].ctrl.vel_set 	= 0.15f;      
	motor[LMotor2].ctrl.pos_set 	= 0.0f;      
	motor[LMotor2].ctrl.tor_set 	= 0.0f;      
	motor[LMotor2].ctrl.cur_set 	= 0.0f;      
	motor[LMotor2].ctrl.kp_set 	= 0.0f;      
	motor[LMotor2].ctrl.kd_set 	= 0.0f;      
	motor[LMotor2].tmp.PMAX		= 12.5f; 
	motor[LMotor2].tmp.VMAX		= 30.0f; 
	motor[LMotor2].tmp.TMAX		= 10.0f; 

	// 设置Motor3的电机信息
	motor[LMotor3].id = 0x03; 
	motor[LMotor3].mst_id = 0x00;	
	motor[LMotor3].tmp.read_flag = 1;
	motor[LMotor3].ctrl.mode 	= pos_mode;  // 改为位置模式
	motor[LMotor3].ctrl.vel_set 	= 1.0f;      
	motor[LMotor3].ctrl.pos_set 	= 0.0f;      
	motor[LMotor3].ctrl.tor_set 	= 0.0f;      
	motor[LMotor3].ctrl.cur_set 	= 0.0f;      
	motor[LMotor3].ctrl.kp_set 	= 0.0f;      
	motor[LMotor3].ctrl.kd_set 	= 0.0f;      
	motor[LMotor3].tmp.PMAX		= 12.5f; 
	motor[LMotor3].tmp.VMAX		= 30.0f; 
	motor[LMotor3].tmp.TMAX		= 10.0f; 

	// 设置Motor4的电机信息
	motor[LMotor4].id = 0x04; 
	motor[LMotor4].mst_id = 0x00;	
	motor[LMotor4].tmp.read_flag = 1;
	motor[LMotor4].ctrl.mode 	= pos_mode;
	motor[LMotor4].ctrl.vel_set 	= 1.0f;  
	motor[LMotor4].ctrl.pos_set 	= 0.0f;      // 统一设为0.0f，如需3.14f请自行修改
	motor[LMotor4].ctrl.tor_set 	= 0.0f;  
	motor[LMotor4].ctrl.cur_set 	= 0.0f;  
	motor[LMotor4].ctrl.kp_set 	= 0.0f;  
	motor[LMotor4].ctrl.kd_set 	= 0.0f;  
	motor[LMotor4].tmp.PMAX		= 12.5f; 
	motor[LMotor4].tmp.VMAX		= 30.0f; 
	motor[LMotor4].tmp.TMAX		= 10.0f; 
	
	// 设置Motor5的电机信息
	motor[LMotor5].id = 0x05; 
	motor[LMotor5].mst_id = 0x00;	
	motor[LMotor5].tmp.read_flag = 1;
	motor[LMotor5].ctrl.mode 	= pos_mode;
	motor[LMotor5].ctrl.vel_set 	= 1.0f;  
	motor[LMotor5].ctrl.pos_set 	= 0.0f;  
	motor[LMotor5].ctrl.tor_set 	= 0.0f;  
	motor[LMotor5].ctrl.cur_set 	= 0.0f;  
	motor[LMotor5].ctrl.kp_set 	= 0.0f;  
	motor[LMotor5].ctrl.kd_set 	= 0.0f;  
	motor[LMotor5].tmp.PMAX		= 12.5f; 
	motor[LMotor5].tmp.VMAX		= 30.0f; 
	motor[LMotor5].tmp.TMAX		= 10.0f; 

	// 设置LMotor6的电机信息
	motor[LMotor6].id = 0x06; 
	motor[LMotor6].mst_id = 0x00;	
	motor[LMotor6].tmp.read_flag = 1;
	motor[LMotor6].ctrl.mode 	= pos_mode;
	motor[LMotor6].ctrl.vel_set 	= 1.0f;  
	motor[LMotor6].ctrl.pos_set 	= 0.0f;  
	motor[LMotor6].ctrl.tor_set 	= 0.0f;  
	motor[LMotor6].ctrl.cur_set 	= 0.0f;  
	motor[LMotor6].ctrl.kp_set 	= 0.0f;  
	motor[LMotor6].ctrl.kd_set 	= 0.0f;  
	motor[LMotor6].tmp.PMAX		= 12.5f; 
	motor[LMotor6].tmp.VMAX		= 30.0f; 
	motor[LMotor6].tmp.TMAX		= 10.0f; 
				
	// --- 新增设置Motor7的电机信息 ---
  motor[LMotor7].id = 0x07; 
	motor[LMotor7].mst_id = 0x00;	
	motor[LMotor7].tmp.read_flag = 1;
	motor[LMotor7].ctrl.mode 	= pos_mode;
	motor[LMotor7].ctrl.vel_set 	= 1.0f;  
	motor[LMotor7].ctrl.pos_set 	= 0.0f;  
	motor[LMotor7].ctrl.tor_set 	= 0.0f;  
	motor[LMotor7].ctrl.cur_set 	= 0.0f;  
	motor[LMotor7].ctrl.kp_set 	= 0.0f;  
	motor[LMotor7].ctrl.kd_set 	= 0.0f;  
	motor[LMotor7].tmp.PMAX		= 12.5f; 
	motor[LMotor7].tmp.VMAX		= 30.0f; 
	motor[LMotor7].tmp.TMAX		= 10.0f; 
				
	// --- 新增设置Motor8的电机信息 ---
	motor[LMotor8].id = 0x08; 
	motor[LMotor8].mst_id = 0x00;	
	motor[LMotor8].tmp.read_flag = 1;
	motor[LMotor8].ctrl.mode 	= pos_mode;
	motor[LMotor8].ctrl.vel_set 	= 1.0f;  
	motor[LMotor8].ctrl.pos_set 	= 0.0f;  
	motor[LMotor8].ctrl.tor_set 	= 0.0f;  
	motor[LMotor8].ctrl.cur_set 	= 0.0f;  
	motor[LMotor8].ctrl.kp_set 	= 0.0f;  
	motor[LMotor8].ctrl.kd_set 	= 0.0f;  
	motor[LMotor8].tmp.PMAX		= 12.5f; 
	motor[LMotor8].tmp.VMAX		= 30.0f; 
	motor[LMotor8].tmp.TMAX		= 10.0f;
				
	// --- 新增设置Motor9的电机信息 ---
	motor[LMotor9].id = 0x09; 
	motor[LMotor9].mst_id = 0x00;
	motor[LMotor9].tmp.read_flag = 1;
	motor[LMotor9].ctrl.mode 	= pos_mode;
	motor[LMotor9].ctrl.vel_set 	= 0.15f;  
	motor[LMotor9].ctrl.pos_set 	= 0.0f;  
	motor[LMotor9].ctrl.tor_set 	= 0.0f;  
	motor[LMotor9].ctrl.cur_set 	= 0.0f;  
	motor[LMotor9].ctrl.kp_set 	= 0.0f;  
	motor[LMotor9].ctrl.kd_set 	= 0.0f;  
	motor[LMotor9].tmp.PMAX		= 12.5f; 
	motor[LMotor9].tmp.VMAX		= 30.0f; 
	motor[LMotor9].tmp.TMAX		= 10.0f;
	
	// 设置RMotor1的电机信息
	motor[RMotor1].id = 0x01;
	motor[RMotor1].mst_id = 0x00;	
	motor[RMotor1].tmp.read_flag = 1;
	motor[RMotor1].ctrl.mode 	= pos_mode;  // 改为位置模式
	motor[RMotor1].ctrl.vel_set 	= 0.18f;      // 位置模式需要给运行速度
	motor[RMotor1].ctrl.pos_set 	= 0.0f;      
	motor[RMotor1].ctrl.tor_set 	= 0.0f;      // 位置模式下通常扭矩设为0
	motor[RMotor1].ctrl.cur_set 	= 0.0f;      
	motor[RMotor1].ctrl.kp_set 	= 0.0f;      
	motor[RMotor1].ctrl.kd_set 	= 0.0f;      
	motor[RMotor1].tmp.PMAX		= 12.5f; 
	motor[RMotor1].tmp.VMAX		= 30.0f; 
	motor[RMotor1].tmp.TMAX		= 10.0f; 
				
	// 设置RMotor2的电机信息
	motor[RMotor2].id = 0x02; 
	motor[RMotor2].mst_id = 0x00;	
	motor[RMotor2].tmp.read_flag = 1;
	motor[RMotor2].ctrl.mode 	= pos_mode;  // 改为位置模式
	motor[RMotor2].ctrl.vel_set 	= 0.15f;      
	motor[RMotor2].ctrl.pos_set 	= 0.0f;      
	motor[RMotor2].ctrl.tor_set 	= 0.0f;      
	motor[RMotor2].ctrl.cur_set 	= 0.0f;      
	motor[RMotor2].ctrl.kp_set 	= 0.0f;      
	motor[RMotor2].ctrl.kd_set 	= 0.0f;      
	motor[RMotor2].tmp.PMAX		= 12.5f; 
	motor[RMotor2].tmp.VMAX		= 30.0f; 
	motor[RMotor2].tmp.TMAX		= 10.0f; 
				
	// 设置RMotor3的电机信息
	motor[RMotor3].id = 0x03; 
	motor[RMotor3].mst_id = 0x00;	
	motor[RMotor3].tmp.read_flag = 1;
	motor[RMotor3].ctrl.mode 	= pos_mode;  // 改为位置模式
	motor[RMotor3].ctrl.vel_set 	= 1.0f;      
	motor[RMotor3].ctrl.pos_set 	= 0.0f;      
	motor[RMotor3].ctrl.tor_set 	= 0.0f;      
	motor[RMotor3].ctrl.cur_set 	= 0.0f;      
	motor[RMotor3].ctrl.kp_set 	= 0.0f;      
	motor[RMotor3].ctrl.kd_set 	= 0.0f;      
	motor[RMotor3].tmp.PMAX		= 12.5f; 
	motor[RMotor3].tmp.VMAX		= 30.0f; 
	motor[RMotor3].tmp.TMAX		= 10.0f; 
				
	// 设置RMotor4的电机信息
	motor[RMotor4].id = 0x04; 
	motor[RMotor4].mst_id = 0x00;	
	motor[RMotor4].tmp.read_flag = 1;
	motor[RMotor4].ctrl.mode 	= pos_mode;
	motor[RMotor4].ctrl.vel_set 	= 1.0f;  
	motor[RMotor4].ctrl.pos_set 	= 0.0f;      // 统一设为0.0f，如需3.14f请自行修改
	motor[RMotor4].ctrl.tor_set 	= 0.0f;  
	motor[RMotor4].ctrl.cur_set 	= 0.0f;  
	motor[RMotor4].ctrl.kp_set 	= 0.0f;  
	motor[RMotor4].ctrl.kd_set 	= 0.0f;  
	motor[RMotor4].tmp.PMAX		= 12.5f; 
	motor[RMotor4].tmp.VMAX		= 30.0f; 
	motor[RMotor4].tmp.TMAX		= 10.0f; 
				
	// 设置RMotor5的电机信息
	motor[RMotor5].id = 0x05; 
	motor[RMotor5].mst_id = 0x00;	
	motor[RMotor5].tmp.read_flag = 1;
	motor[RMotor5].ctrl.mode 	= pos_mode;
	motor[RMotor5].ctrl.vel_set 	= 1.0f;  
	motor[RMotor5].ctrl.pos_set 	= 0.0f;  
	motor[RMotor5].ctrl.tor_set 	= 0.0f;  
	motor[RMotor5].ctrl.cur_set 	= 0.0f;  
	motor[RMotor5].ctrl.kp_set 	= 0.0f;  
	motor[RMotor5].ctrl.kd_set 	= 0.0f;  
	motor[RMotor5].tmp.PMAX		= 12.5f; 
	motor[RMotor5].tmp.VMAX		= 30.0f; 
	motor[RMotor5].tmp.TMAX		= 10.0f; 
				
	// 设置RMotor6的电机信息
	motor[RMotor6].id = 0x06; 
	motor[RMotor6].mst_id = 0x00;	
	motor[RMotor6].tmp.read_flag = 1;
	motor[RMotor6].ctrl.mode 	= pos_mode;
	motor[RMotor6].ctrl.vel_set 	= 1.0f;  
	motor[RMotor6].ctrl.pos_set 	= 0.0f;  
	motor[RMotor6].ctrl.tor_set 	= 0.0f;  
	motor[RMotor6].ctrl.cur_set 	= 0.0f;  
	motor[RMotor6].ctrl.kp_set 	= 0.0f;  
	motor[RMotor6].ctrl.kd_set 	= 0.0f;  
	motor[RMotor6].tmp.PMAX		= 12.5f; 
	motor[RMotor6].tmp.VMAX		= 30.0f; 
	motor[RMotor6].tmp.TMAX		= 10.0f; 
				
	// --- 新增设置Motor7的电机信息 ---
  motor[RMotor7].id = 0x07; 
	motor[RMotor7].mst_id = 0x00;	
	motor[RMotor7].tmp.read_flag = 1;
	motor[RMotor7].ctrl.mode 	= pos_mode;
	motor[RMotor7].ctrl.vel_set 	= 1.0f;  
	motor[RMotor7].ctrl.pos_set 	= 0.0f;  
	motor[RMotor7].ctrl.tor_set 	= 0.0f;  
	motor[RMotor7].ctrl.cur_set 	= 0.0f;  
	motor[RMotor7].ctrl.kp_set 	= 0.0f;  
	motor[RMotor7].ctrl.kd_set 	= 0.0f;  
	motor[RMotor7].tmp.PMAX		= 12.5f; 
	motor[RMotor7].tmp.VMAX		= 30.0f; 
	motor[RMotor7].tmp.TMAX		= 10.0f; 
				
	// --- 新增设置Motor8的电机信息 ---
	motor[RMotor8].id = 0x08; 
	motor[RMotor8].mst_id = 0x00;	
	motor[RMotor8].tmp.read_flag = 1;
	motor[RMotor8].ctrl.mode 	= pos_mode;
	motor[RMotor8].ctrl.vel_set 	= 1.0f;  
	motor[RMotor8].ctrl.pos_set 	= 0.0f;  
	motor[RMotor8].ctrl.tor_set 	= 0.0f;  
	motor[RMotor8].ctrl.cur_set 	= 0.0f;  
	motor[RMotor8].ctrl.kp_set 	= 0.0f;  
	motor[RMotor8].ctrl.kd_set 	= 0.0f;  
	motor[RMotor8].tmp.PMAX		= 12.5f; 
	motor[RMotor8].tmp.VMAX		= 30.0f; 
	motor[RMotor8].tmp.TMAX		= 10.0f;
}
/**
************************************************************************
* @brief:      	read_all_motor_data: 读取电机的所有寄存器的数据信息
* @param:      	motor_t：电机参数结构体
* @retval:     	void
* @details:    	逐次发送读取命令
************************************************************************
**/
void read_all_motor_data(motor_t *motor)
{
    switch (motor->tmp.read_flag)
    {
		case 1:  read_motor_data(motor->id, RID_UV_VALUE);  break; // UV_Value
		case 2:  read_motor_data(motor->id, RID_KT_VALUE);  break; // KT_Value
		case 3:  read_motor_data(motor->id, RID_OT_VALUE);  break; // OT_Value
		case 4:  read_motor_data(motor->id, RID_OC_VALUE);  break; // OC_Value
		case 5:  read_motor_data(motor->id, RID_ACC);       break; // ACC
		case 6:  read_motor_data(motor->id, RID_DEC);       break; // DEC
		case 7:  read_motor_data(motor->id, RID_MAX_SPD);   break; // MAX_SPD
		case 8:  read_motor_data(motor->id, RID_MST_ID);    break; // MST_ID 
		case 9:  read_motor_data(motor->id, RID_ESC_ID);    break; // ESC_ID
		case 10: read_motor_data(motor->id, RID_TIMEOUT);   break; // TIMEOUT 
		case 11: read_motor_data(motor->id, RID_CMODE);     break; // CTRL_MODE 
		case 12: read_motor_data(motor->id, RID_DAMP);      break; // Damp 
		case 13: read_motor_data(motor->id, RID_INERTIA);   break; // Inertia
		case 14: read_motor_data(motor->id, RID_HW_VER);    break; // Rsv1 
		case 15: read_motor_data(motor->id, RID_SW_VER);    break; // sw_ver 
		case 16: read_motor_data(motor->id, RID_SN);        break; // Rsv2 
		case 17: read_motor_data(motor->id, RID_NPP);       break; // NPP 
		case 18: read_motor_data(motor->id, RID_RS);        break; // Rs 
		case 19: read_motor_data(motor->id, RID_LS);        break; // Ls 
		case 20: read_motor_data(motor->id, RID_FLUX);      break; // Flux 
		case 21: read_motor_data(motor->id, RID_GR);        break; // Gr 
		case 22: read_motor_data(motor->id, RID_PMAX);      break; // PMAX 
		case 23: read_motor_data(motor->id, RID_VMAX);      break; // VMAX 
		case 24: read_motor_data(motor->id, RID_TMAX);      break; // TMAX 
		case 25: read_motor_data(motor->id, RID_I_BW);      break; // I_BW 
		case 26: read_motor_data(motor->id, RID_KP_ASR);    break; // KP_ASR 
		case 27: read_motor_data(motor->id, RID_KI_ASR);    break; // KI_ASR 
		case 28: read_motor_data(motor->id, RID_KP_APR);    break; // KP_APR 
		case 29: read_motor_data(motor->id, RID_KI_APR);    break; // KI_APR 
		case 30: read_motor_data(motor->id, RID_OV_VALUE);  break; // OV_Value 
		case 31: read_motor_data(motor->id, RID_GREF);      break; // GREF 
		case 32: read_motor_data(motor->id, RID_DETA);      break; // Deta 
		case 33: read_motor_data(motor->id, RID_V_BW);      break; // V_BW 
		case 34: read_motor_data(motor->id, RID_IQ_CL);     break; // IQ_c1 
		case 35: read_motor_data(motor->id, RID_VL_CL);     break; // VL_c1 
		case 36: read_motor_data(motor->id, RID_CAN_BR);    break; // can_br 
		case 37: read_motor_data(motor->id, RID_SUB_VER);   break; // sub_ver 
		case 38: read_motor_data(motor->id, RID_U_OFF);     break; // u_off 
		case 39: read_motor_data(motor->id, RID_V_OFF);     break; // v_off 
		case 40: read_motor_data(motor->id, RID_K1);        break; // k1 
		case 41: read_motor_data(motor->id, RID_K2);        break; // k2 
		case 42: read_motor_data(motor->id, RID_M_OFF);     break; // m_off 
		case 43: read_motor_data(motor->id, RID_DIR);       break; // dir 
		case 44: read_motor_data(motor->id, RID_P_M);       break; // pm 
		case 45: read_motor_data(motor->id, RID_X_OUT);     break; // xout 
    }
}
/**
************************************************************************
* @brief:      	receive_motor_data: 接收电机返回的数据信息
* @param:      	motor_t：电机参数结构体
* @param:      	data：接收的数据
* @retval:     	void
* @details:    	逐次接收电机回传的参数信息
************************************************************************
**/
void receive_motor_data(motor_t *motor, uint8_t *data)
{
	if(motor->tmp.read_flag == 0)
		return ;
	
	float_type_u y;
	
	if(data[2] == 0x33)
	{
		uint16_t rid_value = data[3];
		y.b_val[0] = data[4];
		y.b_val[1] = data[5];
		y.b_val[2] = data[6];
		y.b_val[3] = data[7];
		
		switch (rid_value) 
		{
			case RID_UV_VALUE: motor->tmp.UV_Value = y.f_val; motor->tmp.read_flag =  2; break;
			case RID_KT_VALUE: motor->tmp.KT_Value = y.f_val; motor->tmp.read_flag =  3; break;
			case RID_OT_VALUE: motor->tmp.OT_Value = y.f_val; motor->tmp.read_flag =  4; break;
			case RID_OC_VALUE: motor->tmp.OC_Value = y.f_val; motor->tmp.read_flag =  5; break;
			case RID_ACC:      motor->tmp.ACC      = y.f_val; motor->tmp.read_flag =  6; break;
			case RID_DEC:      motor->tmp.DEC      = y.f_val; motor->tmp.read_flag =  7; break;
			case RID_MAX_SPD:  motor->tmp.MAX_SPD  = y.f_val; motor->tmp.read_flag =  8; break;
			case RID_MST_ID:   motor->tmp.MST_ID   = y.u_val; motor->tmp.read_flag =  9; break;
			case RID_ESC_ID:   motor->tmp.ESC_ID   = y.u_val; motor->tmp.read_flag = 10; break;
			case RID_TIMEOUT:  motor->tmp.TIMEOUT  = y.u_val; motor->tmp.read_flag = 11; break;
			case RID_CMODE:    motor->tmp.cmode    = y.u_val; motor->tmp.read_flag = 12; break;
			case RID_DAMP:     motor->tmp.Damp     = y.f_val; motor->tmp.read_flag = 13; break;
			case RID_INERTIA:  motor->tmp.Inertia  = y.f_val; motor->tmp.read_flag = 14; break;
			case RID_HW_VER:   motor->tmp.hw_ver   = y.u_val; motor->tmp.read_flag = 15; break;
			case RID_SW_VER:   motor->tmp.sw_ver   = y.u_val; motor->tmp.read_flag = 16; break;
			case RID_SN:       motor->tmp.SN       = y.u_val; motor->tmp.read_flag = 17; break;
			case RID_NPP:      motor->tmp.NPP      = y.u_val; motor->tmp.read_flag = 18; break;
			case RID_RS:       motor->tmp.Rs       = y.f_val; motor->tmp.read_flag = 19; break;
			case RID_LS:       motor->tmp.Ls       = y.f_val; motor->tmp.read_flag = 20; break;
			case RID_FLUX:     motor->tmp.Flux     = y.f_val; motor->tmp.read_flag = 21; break;
			case RID_GR:       motor->tmp.Gr       = y.f_val; motor->tmp.read_flag = 22; break;
			case RID_PMAX:     motor->tmp.PMAX     = y.f_val; motor->tmp.read_flag = 23; break;
			case RID_VMAX:     motor->tmp.VMAX     = y.f_val; motor->tmp.read_flag = 24; break;
			case RID_TMAX:     motor->tmp.TMAX     = y.f_val; motor->tmp.read_flag = 25; break;
			case RID_I_BW:     motor->tmp.I_BW     = y.f_val; motor->tmp.read_flag = 26; break;
			case RID_KP_ASR:   motor->tmp.KP_ASR   = y.f_val; motor->tmp.read_flag = 27; break;
			case RID_KI_ASR:   motor->tmp.KI_ASR   = y.f_val; motor->tmp.read_flag = 28; break;
			case RID_KP_APR:   motor->tmp.KP_APR   = y.f_val; motor->tmp.read_flag = 29; break;
			case RID_KI_APR:   motor->tmp.KI_APR   = y.f_val; motor->tmp.read_flag = 30; break;
			case RID_OV_VALUE: motor->tmp.OV_Value = y.f_val; motor->tmp.read_flag = 31; break;
			case RID_GREF:     motor->tmp.GREF     = y.f_val; motor->tmp.read_flag = 32; break;
			case RID_DETA:     motor->tmp.Deta     = y.f_val; motor->tmp.read_flag = 33; break;
			case RID_V_BW:     motor->tmp.V_BW     = y.f_val; motor->tmp.read_flag = 34; break;
			case RID_IQ_CL:    motor->tmp.IQ_cl    = y.f_val; motor->tmp.read_flag = 35; break;
			case RID_VL_CL:    motor->tmp.VL_cl    = y.f_val; motor->tmp.read_flag = 36; break;
			case RID_CAN_BR:   motor->tmp.can_br   = y.u_val; motor->tmp.read_flag = 37; break;
			case RID_SUB_VER:  motor->tmp.sub_ver  = y.u_val; motor->tmp.read_flag = 38; break;
			case RID_U_OFF:    motor->tmp.u_off    = y.f_val; motor->tmp.read_flag = 39; break;
			case RID_V_OFF:    motor->tmp.v_off    = y.f_val; motor->tmp.read_flag = 40; break;
			case RID_K1:       motor->tmp.k1       = y.f_val; motor->tmp.read_flag = 41; break;
			case RID_K2:       motor->tmp.k2       = y.f_val; motor->tmp.read_flag = 42; break;
			case RID_M_OFF:    motor->tmp.m_off    = y.f_val; motor->tmp.read_flag = 43; break;
			case RID_DIR:      motor->tmp.dir      = y.f_val; motor->tmp.read_flag = 44; break;
			case RID_P_M:      motor->tmp.p_m      = y.f_val; motor->tmp.read_flag = 45; break;
			case RID_X_OUT:    motor->tmp.x_out    = y.f_val; motor->tmp.read_flag = 0 ; break;
		}
	}
}

/**
************************************************************************
* @brief:      	fdcan1_rx_callback: CAN1接收回调函数
* @param:      	void
* @retval:     	void
* @details:    	处理CAN1接收中断回调，根据接收到的ID和数据，执行相应的处理。
*               当接收到ID为0时，调用dm4310_fbdata函数更新Motor的反馈数据。
************************************************************************
**/
void fdcan1_rx_callback(void)
{
	uint16_t rec_id;
	uint8_t rx_data[8] = {0};
	fdcanx_receive(&hfdcan1, &rec_id, rx_data);
	switch (rec_id)
	{
 		case 0x00: dm_motor_fbdata(&motor[LMotor1], rx_data); receive_motor_data(&motor[LMotor1], rx_data); break;
	}
}
