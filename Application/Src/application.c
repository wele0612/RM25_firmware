#include<application.h>

#include <buzzer.h>
#include <h7can.h>
#include <icm42688.h>
#include <receiver.h>
#include <servo_pwm.h>

#include <robot_arch.h>

#define GENERATE_DECLARE_GLOBAL_VARS
#include <global_variables.h>
#include <btb.h>
#undef GENERATE_DECLARE_GLOBAL_VARS

timeout_monitor_t timeout;

// DMA buffer outside
RAM_D2_SECTION uint8_t dr16_buffer_recv[32]; // 18 bytes total
RAM_D2_SECTION uint8_t referee_dma_buf[32];
RAM_D2_SECTION uint8_t aiming_dma_buf[32];
RAM_D2_SECTION uint8_t vtm_dma_buf[32];

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

    if (err) {
        robot_config.imu_gyro_offset[0] = 0.0f;
        robot_config.imu_gyro_offset[1] = 0.0f;
        robot_config.imu_gyro_offset[2] = 0.0f;
    }

    // while(err){
    //     buzzer_calibration_startup();
    // }
}

int remote_online(){
    return (HAL_GetTick() - timeout.last_remote_tick) < REMOTE_TIMEOUT;
}

void robot_init(){
    imu_state=IMU_RESET;

    // Must give IMU clock first, so that IMU does not go into sleep mode.
    HAL_TIM_Base_Start_IT(&htim6);// BUG: IMU_INT1 does not work. Use TIM6 interruption for now...
    can_bsp_init();
    HAL_UARTEx_ReceiveToIdle_DMA(DR16_UART, dr16_buffer_recv, 32);
    HAL_UART_Receive_DMA(REFEREE_UART, referee_dma_buf, 32);
    __HAL_UART_ENABLE_IT(REFEREE_UART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(AIMING_UART, aiming_dma_buf, 32);
    __HAL_UART_ENABLE_IT(AIMING_UART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(VTM_UART, vtm_dma_buf, 32);
    __HAL_UART_ENABLE_IT(VTM_UART, UART_IT_IDLE);

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

    servo_init();
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
    uint16_t robot_id = referee.robot_status_0x0201.robot_id;
    ui_data.sender_id = robot_id;
    ui_data.receiver_id = get_client_id(robot_id);

    const int x_offset = 0;
    const int y_offset = 50 - (HAL_GetTick()/10)%200;

    const int firing_table[7][3]={ 
        //  x   y   length   
            3,-90,180,
            3,-66, 120,
            3,-58,70,
            3,-63,45,
            3,  -80,27,
            3,  -100,20,
            5,  -140,13
        };
        const char firing_table_names[7][3]={
            "f0", "f1", "f2", "f3", "f4", "f5", "f6"
        };

    size_t content_length;
    uint8_t *send_buf;

    figure_operation_t ui_element_op = FIGURE_OPERATION_MODIFY;
    if(updata_level == 0){
        ui_element_op = FIGURE_OPERATION_ADD;
    }

    #define referee_send_frame() do{\
        send_buf = referee_send_data(6 + content_length, &ui_data);\
        HAL_UART_Transmit_DMA(REFEREE_UART, send_buf, 15+content_length);\
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
        const char* helloworld = "PRESS  Q  SPINTOP";
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
        const char* str3 = "R  RESET UI";
        strcpy((char *)ui_data.user_data.char_graphic.char_data, str3);
        draw_char(&(ui_data.user_data.char_graphic.graphic_data), "ct3", ui_element_op, 3,
            COLOR_GREEN, 2, 192, 760, 16, sizeof(str3));

        referee_send_frame();

        memset(ui_data.user_data.raw_data, 0, sizeof(ui_data.user_data.raw_data));
        const char* str4 = "X  TURNBACK";
        strcpy((char *)ui_data.user_data.char_graphic.char_data, str4);
        draw_char(&(ui_data.user_data.char_graphic.graphic_data), "ct4", ui_element_op, 3,
            COLOR_GREEN, 2, 192, 730, 16, sizeof(str4));

        referee_send_frame();

        memset(ui_data.user_data.raw_data, 0, sizeof(ui_data.user_data.raw_data));
        const char* str5 = "B  HOLD TO IGNORE HEAT";
        strcpy((char *)ui_data.user_data.char_graphic.char_data, str5);
        draw_char(&(ui_data.user_data.char_graphic.graphic_data), "ct5", ui_element_op, 3,
            COLOR_GREEN, 2, 192, 700, 16, sizeof(str5));

        referee_send_frame();

        ui_data.data_cmd_id = CMD_DRAW_SEVEN_GRAPHICS;
        content_length = sizeof(interaction_figure_t)*7;

        memset(ui_data.user_data.raw_data, 0, sizeof(ui_data.user_data.raw_data));

        for(int i=0;i<7;i++){
            draw_line(&(ui_data.user_data.seven_graphics[i]), firing_table_names[i], ui_element_op, 4,
            (i<2) ? COLOR_YELLOW : COLOR_GREEN, 
            2, 
            1920/2 - firing_table[i][2]/2 + firing_table[i][0],
            1080/2 + firing_table[i][1],
            1920/2 + firing_table[i][2]/2 + firing_table[i][0],
            1080/2 + firing_table[i][1]);
        }

        referee_send_frame();

    }

    if(updata_level <= 1){
        ui_data.data_cmd_id = CMD_DRAW_SEVEN_GRAPHICS;
        content_length = sizeof(interaction_figure_t)*7;

        memset(ui_data.user_data.raw_data, 0, sizeof(ui_data.user_data.raw_data));
        //竖准心
        draw_line(&(ui_data.user_data.seven_graphics[0]), "zxv", ui_element_op, 2,
            COLOR_ORANGE, 1, 960 + firing_table[3][0], 1080/2 + 100,
                960 + firing_table[3][0], 1080/2 - 200);

        draw_rectangle(&(ui_data.user_data.seven_graphics[1]), "sel", ui_element_op, 1,
            COLOR_PINK, 6, 181, 795, 182+32, 795+32);


        referee_send_frame();
    }

    ui_data.data_cmd_id = CMD_DRAW_SEVEN_GRAPHICS;
    content_length = sizeof(interaction_figure_t)*7;

    memset(ui_data.user_data.raw_data, 0, sizeof(ui_data.user_data.raw_data));

    uint32_t gimbal_rotation_deg = (uint32_t)(wrap_to_2pi(b2g_B.gimbal_mtr_yaw_pos*1e-4f)*RADtoDEG);

    // 车体旋转位置的小条
    draw_arc(&(ui_data.user_data.seven_graphics[0]), "bdy", ui_element_op, 1,
        COLOR_PINK, 5, 1920/2, 1080/2, (gimbal_rotation_deg+345)%360, (gimbal_rotation_deg +15)%360,
        82, 82);
    
    //电容能量条
    #ifdef CONFIG_PLATFORM_BASE

    figure_color_t cap_color = COLOR_YELLOW;
    if(supercap_online() && supercap.cap_state == CAP_ON){
        cap_color = COLOR_GREEN;
    }
    draw_arc(&(ui_data.user_data.seven_graphics[1]), "cp", ui_element_op, 1, 
        cap_color, 8, 960, 536, 219, 220+supercap.cap_energy_percentage, 340, 350);
    #else
    draw_arc(&(ui_data.user_data.seven_graphics[1]), "cp", ui_element_op, 1, 
        COLOR_YELLOW, 8, 960, 536, 220, 320, 340, 350);
    #endif

    //横准心
    // draw_line(&(ui_data.user_data.seven_graphics[2]), "zxh", ui_element_op, 2,
    // COLOR_GREEN, 3, 960 - 30 + x_offset, 1080/2 + y_offset,
    //     960 + 30 + x_offset, 1080/2 + y_offset);

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

    // 自瞄范围的框
    draw_rectangle(&(ui_data.user_data.seven_graphics[5]), "aim", ui_element_op, 2,
        chasis_ctrl.minipc_online ? COLOR_TEAM : COLOR_PURPLE, 5, 
            1920/2 - 300, 1080/2 - 230, 
            1920/2 + 300, 1080/2 + 230);

    referee_send_frame();

    #undef referee_send_frame
}


void robot_step(const float CTRL_DELTA_T){
    //HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, SET);

    imu_obtain_data(&imu_data);

    controller_cycle(CTRL_DELTA_T);

    HAL_UART_Transmit_DMA(&huart7, (void *)&vofa, sizeof(vofa));

    //HAL_GPIO_WritePin(LED_PC13_GPIO_Port, LED_PC13_Pin, RESET);
}

void robot_loop(){
    //HAL_Delay(2000);
    //why_play_harunokage();
    if(chasis_ctrl.custom_UI_drawcall){
        referee_ui_update(0);
    }else{
        referee_ui_update(2);
    }
    check_and_recover_all_can();
}

static uint16_t referee_dma_read_idx = 0;
static uint16_t aiming_dma_read_idx = 0;
static uint16_t vtm_dma_read_idx = 0;

#define PROCESS_CIRCULAR_DMA(huart, buf, read_idx, recv_fn) do{ \
    uint16_t ndtr = __HAL_DMA_GET_COUNTER((huart)->hdmarx); \
    uint16_t write_idx = 32 - ndtr; \
    while((read_idx) != write_idx){ \
        (recv_fn)((buf)[(read_idx)]); \
        (read_idx) = ((read_idx) + 1) % 32; \
    } \
}while(0)

void robot_UART_msgcallback(UART_HandleTypeDef *huart, HAL_UART_RxEventTypeTypeDef event){
    
    if(huart == DR16_UART){
        if(event == HAL_UART_RXEVENT_HT) return;

        HAL_UARTEx_ReceiveToIdle_DMA(DR16_UART, dr16_buffer_recv, 32); // 接收完毕后重启
        parse_DR16_receiver_msg(&dr16, dr16_buffer_recv);

    }else if(huart == REFEREE_UART){
        PROCESS_CIRCULAR_DMA(huart, referee_dma_buf, referee_dma_read_idx, referee_recv_byte);
    }else if(huart == AIMING_UART){
        #ifdef CONFIG_ENABLE_VISION
        PROCESS_CIRCULAR_DMA(huart, aiming_dma_buf, aiming_dma_read_idx, vision_recv_byte);
        #endif
    }else if(huart == VTM_UART){
        uint16_t ndtr = __HAL_DMA_GET_COUNTER(huart->hdmarx);
        uint16_t write_idx = 32 - ndtr;
        while(vtm_dma_read_idx != write_idx){
            VTM_recv_byte(&vtm, vtm_dma_buf[vtm_dma_read_idx]);
            vtm_dma_read_idx = (vtm_dma_read_idx + 1) % 32;
        }
    }

}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart){
    robot_UART_msgcallback(huart, HAL_UART_RXEVENT_HT);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    robot_UART_msgcallback(huart, HAL_UART_RXEVENT_TC);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    robot_UART_msgcallback(huart, HAL_UARTEx_GetRxEventType(huart));
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	if(huart == DR16_UART)
	{
		__HAL_UNLOCK(huart);
		HAL_UARTEx_ReceiveToIdle_DMA(DR16_UART, dr16_buffer_recv, 32);
	}
	else if(huart == REFEREE_UART)
	{
		__HAL_UNLOCK(huart);
		HAL_UART_DMAStop(REFEREE_UART);
		HAL_UART_Receive_DMA(REFEREE_UART, referee_dma_buf, 32);
		__HAL_UART_ENABLE_IT(REFEREE_UART, UART_IT_IDLE);
	}
	else if(huart == AIMING_UART)
	{
		__HAL_UNLOCK(huart);
		HAL_UART_DMAStop(AIMING_UART);
		HAL_UART_Receive_DMA(AIMING_UART, aiming_dma_buf, 32);
		__HAL_UART_ENABLE_IT(AIMING_UART, UART_IT_IDLE);
	}
	else if(huart == VTM_UART)
	{
		__HAL_UNLOCK(huart);
		HAL_UART_DMAStop(VTM_UART);
		HAL_UART_Receive_DMA(VTM_UART, vtm_dma_buf, 32);
		__HAL_UART_ENABLE_IT(VTM_UART, UART_IT_IDLE);
	}
}


