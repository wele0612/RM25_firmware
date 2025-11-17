#include<application.h>

#include <buzzer.h>
#include <h7can.h>
#include <icm42688.h>
#include <receiver.h>

#include <robot_arch.h>

#define GENERATE_DECLARE_GLOBAL_VARS
#include <global_variables.h>
#undef GENERATE_DECLARE_GLOBAL_VARS

timeout_monitor_t timeout;

// DMA buffer outside
RAM_D2_SECTION uint8_t dr16_buffer_recv[32]; // 18 bytes total

RAM_D2_SECTION robot_VOFA_report_t vofa = {.tail  = VOFA_TAIL};

static int err=0;
static void self_test(){
    err = 0;
    // Check IMU calibration data
    const float imu_gyro_offset_max = 3.0f;
    for(int i=0;i<3;i++){
        if(isnan(robot_config.imu_gyro_offset[i]))  err=1;
        if(fabs(robot_config.imu_gyro_offset[i])>imu_gyro_offset_max)   err=2;
    }

    while(err){
        buzzer_calibration_startup();
    }
}

void robot_init(){
    imu_state=IMU_RESET;

    // Must give IMU clock first, so that IMU does not go into sleep mode.
    HAL_TIM_Base_Start_IT(&htim6);// BUG: IMU_INT1 does not work. Use TIM6 interruption for now...
    can_bsp_init();
    HAL_UARTEx_ReceiveToIdle_DMA(DR16_UART, dr16_buffer_recv, 32);
    HAL_UART_Receive_IT(REFEREE_UART, referee_buf, 1);

    while(icm_init() != 0); //Init IMU
    robot_readconfig(&robot_config); //Read from flash

    // Hold User key during start up to enter IMU callibration mode.
#define IMU_CALIBRATION_MODE_ENABLE
#ifdef IMU_CALIBRATION_MODE_ENABLE
    if(HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) == RESET){
        HAL_Delay(10);
        if(HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) == RESET){
            buzzer_calibration_startup();
            HAL_Delay(5000);

            imu_state = IMU_CALIBRATE;

            for(int i=0;i<3;i++){
                robot_config.imu_gyro_offset[i]=0.0f;
            }

            HAL_Delay(10);

            float gyro_offset[3]={0.0f,0.0f,0.0f};
            const float calib_sample_nums=2000;
            for(int i=0;i<calib_sample_nums;i++){
                imu_obtain_data(&imu_data);
                gyro_offset[0] += imu_data.gyro[0];
                gyro_offset[1] += imu_data.gyro[1];
                gyro_offset[2] += imu_data.gyro[2];
            }
            robot_config.test_val += 1;
            robot_config.imu_gyro_offset[0] = gyro_offset[0]/calib_sample_nums;
            robot_config.imu_gyro_offset[1] = gyro_offset[1]/calib_sample_nums;
            robot_config.imu_gyro_offset[2] = gyro_offset[2]/calib_sample_nums;
            robot_saveconfig(&robot_config);

            imu_state = IMU_RUNNING;
        }
    }
#endif

    self_test();

    buzzer_DJI_startup();

    timeout.last_remote_tick=HAL_GetTick();
    
    controller_init();
    HAL_TIM_Base_Start_IT(&htim7);

    //vofa message buffer is in NOLOAD section. Manually initialize after power up.
    memset(&vofa,0,sizeof(vofa));
    vofa.tail[0]=0x00;
    vofa.tail[1]=0x00;
    vofa.tail[2]=0x80;
    vofa.tail[3]=0x7f;

    referee_ui_update(0);

}
/** @warning This functions is block, DO NOT call this function in ISR
 * 此函数会阻塞，不要在中断中调用此函数。
 *  @brief Draw client UI 自定义UI绘制
 *  @param update_level Redraw level 绘制等级
 *  0:完全重绘
 *  1:更新所有非静态UI
 *  2:仅更新最关键的实时UI，最快
 */
void referee_ui_update(int updata_level){
    const uint32_t referee_frame_delay = 40;

    robot_interaction_data_t ui_data;
    uint16_t robot_id = 3;
    ui_data.sender_id = robot_id;
    ui_data.receiver_id = get_client_id(robot_id);

    const int x_offset = 0;
    const int y_offset = 50 - (HAL_GetTick()/10)%200;

    size_t content_length;
    uint8_t *send_buf;

    figure_operation_t ui_element_op = FIGURE_OPERATION_MODIFY;
    if(updata_level == 0){
        ui_element_op = FIGURE_OPERATION_ADD;
    }

    #define referee_send_frame() do{\
        send_buf = referee_send_data(6 + content_length, &ui_data);\
        HAL_UART_Transmit(REFEREE_UART, send_buf, 15+content_length, 100);\
        HAL_Delay(referee_frame_delay);\
        }while(0);

    // Note:
    // The name of a graph element should not be greater than 3.
    // The layer is between 0~9
    if(updata_level<=0){
        ui_data.data_cmd_id = CMD_DELETE_GRAPHIC_LAYER;
        content_length = 2;
        ui_data.user_data.delete_layer.delete_type = 2;

        referee_send_frame();
        HAL_Delay(100);
        referee_send_frame();
        HAL_Delay(100);

        ui_data.data_cmd_id = CMD_DRAW_CHARACTER_GRAPHIC;
        content_length = sizeof(ext_client_custom_character_t);

        memset(ui_data.user_data.raw_data, 0, sizeof(ui_data.user_data.raw_data));
        const char* helloworld = "CTRL + Q  TURN";
        strcpy((char *)ui_data.user_data.char_graphic.char_data, helloworld);
        draw_char(&(ui_data.user_data.char_graphic.graphic_data), "ct1", ui_element_op, 3,
            COLOR_GREEN, 2, 80, 820, 16, sizeof(helloworld));
        
        referee_send_frame();
        
        memset(ui_data.user_data.raw_data, 0, sizeof(ui_data.user_data.raw_data));
        const char* str2 = "E  AGI BACK";
        strcpy((char *)ui_data.user_data.char_graphic.char_data, str2);
        draw_char(&(ui_data.user_data.char_graphic.graphic_data), "ct2", ui_element_op, 3,
            COLOR_GREEN, 2, 192, 790, 16, sizeof(str2));

        referee_send_frame();

        memset(ui_data.user_data.raw_data, 0, sizeof(ui_data.user_data.raw_data));
        const char* str3 = "D  RESET UI";
        strcpy((char *)ui_data.user_data.char_graphic.char_data, str3);
        draw_char(&(ui_data.user_data.char_graphic.graphic_data), "ct3", ui_element_op, 3,
            COLOR_GREEN, 2, 192, 760, 16, sizeof(str3));

        referee_send_frame();

    }

    if(updata_level <= 1){
        ui_data.data_cmd_id = CMD_DRAW_SEVEN_GRAPHICS;
        content_length = sizeof(interaction_figure_t)*7;

        memset(ui_data.user_data.raw_data, 0, sizeof(ui_data.user_data.raw_data));
        //竖准心
        draw_line(&(ui_data.user_data.seven_graphics[0]), "zxv", ui_element_op, 2,
            COLOR_ORANGE, 1, 960 + x_offset, 1080/2 + 100,
                960 + x_offset, 1080/2 - 200);

        draw_rectangle(&(ui_data.user_data.seven_graphics[1]), "sel", ui_element_op, 1,
            COLOR_PINK, 6, 181, 795, 182+32, 795+32);

        send_buf = referee_send_data(6 + content_length, &ui_data);

        referee_send_frame();
    }

    ui_data.data_cmd_id = CMD_DRAW_SEVEN_GRAPHICS;
    content_length = sizeof(interaction_figure_t)*7;

    memset(ui_data.user_data.raw_data, 0, sizeof(ui_data.user_data.raw_data));

    const uint32_t gimbal_rotation_deg = HAL_GetTick()/10;

    // 车体旋转位置的小条
    draw_arc(&(ui_data.user_data.seven_graphics[0]), "bdy", ui_element_op, 1,
        COLOR_PINK, 5, 1920/2, 1080/2, (gimbal_rotation_deg+345)%360, (gimbal_rotation_deg +15)%360,
        82, 82);
    
    //电容能量条
    draw_arc(&(ui_data.user_data.seven_graphics[1]), "cp", ui_element_op, 1, 
        COLOR_YELLOW, 8, 960, 536, 220, 320, 340, 350);

    //横准心
    draw_line(&(ui_data.user_data.seven_graphics[2]), "zxh", ui_element_op, 2,
    COLOR_GREEN, 3, 960 - 30 + x_offset, 1080/2 + y_offset,
        960 + 30 + x_offset, 1080/2 + y_offset);

    int avaliable_count,low_ammocount;
    if(referee.robot_status_0x0201.robot_id == 0x1 || referee.robot_status_0x0201.robot_id == 0x101){// 是英雄
        avaliable_count = referee.projectile_allowance_0x0208.projectile_allowance_42mm;
        if(avaliable_count <= 0) avaliable_count = 0;
        low_ammocount = (avaliable_count <= 3);
    }else{
        avaliable_count = referee.projectile_allowance_0x0208.projectile_allowance_17mm;
        low_ammocount = (avaliable_count <= 25);
    }
    //可用发弹量警告
    draw_integer(&(ui_data.user_data.seven_graphics[3]), "amc",
                 (low_ammocount && (HAL_GetTick() / 300) % 2 == 0) ? FIGURE_OPERATION_ADD : FIGURE_OPERATION_DELETE, 4,
                 COLOR_ORANGE, 6, 1920 / 2 - 40, 1080 / 2 + 120, 50, avaliable_count);

    //CV当前的锁定目标
    draw_circle(&(ui_data.user_data.seven_graphics[4]), "cv", ui_element_op, 5,
        COLOR_GREEN, 2, 1920/2, 1080/2, 20);

    referee_send_frame();

    #undef referee_send_frame
}


void robot_step(const float CTRL_DELTA_T){
    //HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, SET);

    imu_obtain_data(&imu_data);

    controller_cycle(CTRL_DELTA_T);

    HAL_UART_Transmit_DMA(&huart7, (void *)&vofa, sizeof(vofa));

    //HAL_UART_Transmit_IT(&huart7, (void *)&(vofa), sizeof(vofa));
    //HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, RESET);
}

void robot_loop(){
    //HAL_Delay(2000);
    //why_play_harunokage();
    referee_ui_update(2);
}

void robot_UART_msgcallback(UART_HandleTypeDef *huart){
    
    if(huart == DR16_UART){
        if(HAL_UARTEx_GetRxEventType(DR16_UART) == HAL_UART_RXEVENT_HT){
            return;
        }

        HAL_UARTEx_ReceiveToIdle_DMA(DR16_UART, dr16_buffer_recv, 32); // 接收完毕后重启

        set_DR16_previous_state(&dr16);
        parse_DR16_receiver_msg(&dr16, dr16_buffer_recv);

        timeout.last_remote_tick=HAL_GetTick();
        //HAL_UART_Transmit_DMA(&huart1, (uint8_t *)dr16.msg, 18);
        dr16_on_change();

    // }else if(huart == AIMING_UART){
    //     if(HAL_UARTEx_GetRxEventType(AIMING_UART) == HAL_UART_RXEVENT_HT){
    //         return;
    //     }
    //     parse_aiming_receiver_msg(&aim);

    }else if(huart == REFEREE_UART){
        referee_recv_byte(referee_buf[0]);
        HAL_UART_Receive_IT(REFEREE_UART, referee_buf, 1);

    }

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    robot_UART_msgcallback(huart);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    robot_UART_msgcallback(huart);
    
}

// void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
// 	if(huart == DR16_UART)
// 	{
// 		__HAL_UNLOCK(huart);
// 		HAL_UARTEx_ReceiveToIdle_DMA(DR16_UART, (uint8_t*), 32);
//     }
// }


