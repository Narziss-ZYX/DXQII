/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gpio.h"
#include "GUI.h"
#include <stdio.h>
#include "tim.h"
#include "DS_18B20.h"
#include "MPU6050.h"
#include <string.h>
#include <math.h>
#include "Display_3D.h"
#include "comm.h"
#include "HC05.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern GUI_FLASH const GUI_FONT GUI_FontHZ_KaiTi_20;
extern GUI_FLASH const GUI_FONT GUI_FontHZ_SimSun_16;
extern GUI_CONST_STORAGE GUI_BITMAP bmzyx3;
extern GUI_CONST_STORAGE GUI_BITMAP bmaoyi1;
extern GUI_FLASH const GUI_FONT GUI_FontHZ_SimSun_10;
extern GUI_FLASH const GUI_FONT GUI_FontHZ_SimSun_12;
unsigned char TAB_zheng[128] = {    /* 郑 0xd6a3*/
        ________, ________, ________, ________,
        ________, ________, ________, ________,
        _____X__, ______X_, ________, ________,
        _____XX_, _____XXX, ________, ________,
        ______XX, _____XX_, ____X___, ___XX___,
        ______XX, X____XX_, ____XXXX, XXXXXX__,
        _______X, X___XX__, ____XX__, ___XXX__,
        _______X, X___XX__, ____XX__, __XXX___,
        ________, ____X__X, XX__XX__, __XX____,
        ___XXXXX, XXXXXXXX, XX__XX__, __XX____,
        ________, _XXX____, ____XX__, _XX_____,
        ________, _XXX____, ____XX__, _XX_____,
        ________, _XXX____, ____XX__, _X______,
        ________, _XXX____, ____XX__, XX______,
        ________, _XXX____, ____XX__, X_______,
        ________, _XXX____, XX__XX__, _X______,
        __XXXXXX, XXXXXXXX, XXXXXX__, _XX_____,
        ________, _XX_____, ____XX__, __XX____,
        ________, _XX_____, ____XX__, ___XX___,
        ________, _XX_____, ____XX__, ___XX___,
        ________, _XX_____, ____XX__, ___XX___,
        ________, XXXXX___, ____XX__, ___XX___,
        ________, XX__XXX_, ____XX__, ___XX___,
        _______X, XX___XXX, X___XX__, __XXX___,
        _______X, X_____XX, X___XX_X, XXXXX___,
        ______XX, _______X, XX__XX__, _XXX____,
        _____XX_, _______X, XX__XX__, _XX_____,
        ____XX__, ________, X___XX__, ________,
        ___XX___, ________, ____XX__, ________,
        __XX____, ________, ____XX__, ________,
        ________, ________, ____X___, ________,
        ________, ________, ________, ________
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//#define SHOW_ADLIGHT 2
//#define  SHOW_LINES 3
//#define SHOW_WIFI 4
//#define TEST_LEDAD 0
//#define TEST_ACC 1
//#define TEST_GYRO 2
//uint8_t g_showidx = SHOW_ADLIGHT; //GUI索引显示
//uint8_t g_testidx = TEST_LEDAD; //系统测试功能索引
//uint8_t g_ledstat = 0x01; //流水灯状态
//uint8_t g_mpuok = 0; //陀螺仪初始状态
//
//#define MAX_DATA_LEN 77
//uint8_t g_fax_data[MAX_DATA_LEN];
//uint8_t g_fay_data[MAX_DATA_LEN];
//uint8_t g_faz_data[MAX_DATA_LEN];
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
#define WS_LOGO 0
#define WS_GUI1 1
#define WS_GUI2 2
#define WS_GUI3 3
#define WS_GUI4 4
#define WS_START 5
int g_ws = WS_LOGO;
uint32_t intick = 0;
uint32_t beeptick = 0;

volatile float temp = 0;

uint8_t tempwarn = 0;
uint8_t mpuwarn = 0;
uint32_t warntick = 0;

uint8_t num[4] = {0};
int T_integer = 0;
int T_decimal = 0;
uint8_t page_index = 1;

#define MAX_DATALEN 80

float vTemp[MAX_DATALEN];
int cTemp = 0;
float vPitch[MAX_DATALEN];
int cPitch = 0;
float vRoll[MAX_DATALEN];
int cRoll = 0;
float vYaw[MAX_DATALEN];
int cYaw = 0;
int mpu_interval = 50;

struct SplitFloat {
    int integer;
    int decimal;
};

uint8_t g_bUping = 0;
uint16_t g_upstep = 100;

/* USER CODE END Variables */
/* Definitions for MainTask */
osThreadId_t MainTaskHandle;
const osThreadAttr_t MainTask_attributes = {
        .name = "MainTask",
        .priority = (osPriority_t) osPriorityNormal,
        .stack_size = 512 * 4
};
/* Definitions for KeyTask */
osThreadId_t KeyTaskHandle;
const osThreadAttr_t KeyTask_attributes = {
        .name = "KeyTask",
        .priority = (osPriority_t) osPriorityNormal,
        .stack_size = 512 * 4
};
/* Definitions for UartTask */
osThreadId_t UartTaskHandle;
const osThreadAttr_t UartTask_attributes = {
        .name = "UartTask",
        .priority = (osPriority_t) osPriorityNormal,
        .stack_size = 1024 * 4
};
/* Definitions for GUITask */
osThreadId_t GUITaskHandle;
const osThreadAttr_t GUITask_attributes = {
        .name = "GUITask",
        .priority = (osPriority_t) osPriorityNormal,
        .stack_size = 1024 * 4
};
/* Definitions for GetDataTask */
osThreadId_t GetDataTaskHandle;
const osThreadAttr_t GetDataTask_attributes = {
        .name = "GetDataTask",
        .priority = (osPriority_t) osPriorityNormal,
        .stack_size = 256 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void WSLogo(void);

void DrawLogo(void);

void DrawGUI1(void);

void DrawGUI2(void);

void DrawGUI3(void);

void DrawGUI4(void);


void Beep(int time, int tune);

void DispSeg(uint8_t num[4], uint8_t dot);

void BeepDone(void);

struct SplitFloat split_float(float data);

void Init_HC05(void);

/* USER CODE END FunctionPrototypes */

void StartMainTask(void *argument);

void StartKeyTask(void *argument);

void StartUartTask(void *argument);

void StartGUITask(void *argument);

void StartGetDataTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of MainTask */
    MainTaskHandle = osThreadNew(StartMainTask, NULL, &MainTask_attributes);

    /* creation of KeyTask */
    KeyTaskHandle = osThreadNew(StartKeyTask, NULL, &KeyTask_attributes);

    /* creation of UartTask */
    UartTaskHandle = osThreadNew(StartUartTask, NULL, &UartTask_attributes);

    /* creation of GUITask */
    GUITaskHandle = osThreadNew(StartGUITask, NULL, &GUITask_attributes);

    /* creation of GetDataTask */
    GetDataTaskHandle = osThreadNew(StartGetDataTask, NULL, &GetDataTask_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartMainTask */
/**
  * @brief  Function implementing the MainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMainTask */
void StartMainTask(void *argument) {
    /* USER CODE BEGIN StartMainTask */
    static int mpuWarn_cnt = 0; //警告计数
    HAL_Delay(1000);
    ds18b20_init();
    uint8_t mpuok = MPU_init();
    uint8_t idx = 0;
    uint8_t cnt = 0;
    while (cnt++ < 3 && !mpuok) {
        HAL_Delay(500);
        mpuok = MPU_init();
    }
    uint32_t dstick = 0;
    uint32_t mputick = 0;
    uint32_t uptick = 0;
    /* Infinite loop */
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    for (;;) {
        //温度数据获取
        if (osKernelGetTickCount() >= dstick + mpu_interval) {
            dstick = osKernelGetTickCount();
            float ft = ds18b20_read();
            T_integer = (int) ft;
            T_decimal = (int) ((ft - T_integer) * 100);
            if (ft < 125) {
                temp = ft;
                //温度数据保存
                if (cTemp < MAX_DATALEN)
                    vTemp[cTemp++] = temp;
                else {
                    memcpy((void *) vTemp, (void *) (vTemp + 1), sizeof(vTemp[0]) * (MAX_DATALEN - 1));
                    vTemp[MAX_DATALEN - 1] = temp;
                }
                if (temp >= 35) {
                    tempwarn = 1;
                    printf("temp:%d.%d", T_integer, T_decimal);
                }
            }
        }
        //mpu数据获取
        if (mpuok) {
            if (osKernelGetTickCount() >= mputick + mpu_interval) {
                mputick = osKernelGetTickCount();
                MPU_getdata();
                /*--------------------------温度、姿态数据保存--------------------------*/
                if (cPitch < MAX_DATALEN)
                    vPitch[cPitch++] = fAX;
                else {
                    memcpy((void *) vPitch, (void *) (vPitch + 1), sizeof(vPitch[0]) * (MAX_DATALEN - 1));
                    vPitch[MAX_DATALEN - 1] = fAX;
                }
                if (cRoll < MAX_DATALEN)
                    vRoll[cRoll++] = fAX;
                else {
                    memcpy((void *) vRoll, (void *) (vRoll + 1), sizeof(vRoll[0]) * (MAX_DATALEN - 1));
                    vRoll[MAX_DATALEN - 1] = fAY;
                }
                if (cYaw < MAX_DATALEN)
                    vYaw[cYaw++] = fAX;
                else {
                    memcpy((void *) vYaw, (void *) (vYaw + 1), sizeof(vYaw[0]) * (MAX_DATALEN - 1));
                    vYaw[MAX_DATALEN - 1] = fAZ;
                }
                /*--------------------------震动检测--------------------------*/
                if (gx * gx + gy * gy + gz * gz > 3000) {
                    mpuWarn_cnt++;
                }
                if (mpuWarn_cnt > 15) {
                    mpuwarn = 1;
                    mpuWarn_cnt = 0;
                }
            }
        }
        //报警时30s倒计时
        if (tempwarn || mpuwarn) {
            if (0 == warntick)
                warntick = osKernelGetTickCount();
            else if (osKernelGetTickCount() >= warntick + 30000) {  //30000数值需修改
                tempwarn = mpuwarn = 0;
                warntick = 0;
            } else {
                uint32_t tick = warntick + 30000 - osKernelGetTickCount();
                num[0] = (tick / 10000) % 10;
                num[1] = (tick / 1000) % 10;
                num[2] = (tick / 100) % 10;
                num[3] = (tick / 10) % 10;
            }
            if ((num[2] == 1 || num[2] == 3 || num[2] == 5) && tempwarn) {
                Beep(100, 3);
            } else if ((num[2] == 2 || num[2] == 4) && mpuwarn) {
                Beep(200, 1);
            }
        } else {
            num[0] = num[1] = num[2] = num[3] = ' ';
        }
        if (HC05_IsConn() && g_bUping) {
            if(osKernelGetTickCount()>=uptick+g_upstep)
            {
                uptick = osKernelGetTickCount();
                char buf[100];
                sprintf(buf, "T:%3d.%d,A:%6d %6d %6d,G:%6d %6d %6d,Z:%4d.%d %4d.%d %4d.%d,W:%d\n",
                        split_float(temp).integer, split_float(temp).decimal, ax, ay, az, gx, gy, gz,
                        split_float(fAX).integer, split_float(fAX).decimal, split_float(fAY).integer,
                        split_float(fAY).decimal,
                        split_float(fAZ).integer, split_float(fAZ).decimal, (tempwarn ? 1 : 0) + (mpuwarn ? 2 : 0));
                USendStr(&huart2, (uint8_t *) buf, strlen(buf));
            }
        }
        osDelay(1);
    }
#pragma clang diagnostic pop
    /* USER CODE END StartMainTask */
}

/* USER CODE BEGIN Header_StartKeyTask */
/**
* @brief 按键任务，切换GUI显示
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartKeyTask */
void StartKeyTask(void *argument) {
    /* USER CODE BEGIN StartKeyTask */
    uint32_t keytick = 0;
    /* Infinite loop */
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    for (;;) {
        if (mpuwarn == 0 && tempwarn == 0) {
            //20ms扫描
            if (osKernelGetTickCount() >= keytick + 20) {
                keytick = osKernelGetTickCount();
                uint8_t key = ScanKey();
                if (KEY6 == key)
                    g_ws = WS_START;//返回启动界面
                switch (g_ws) {
                    case WS_LOGO:
                        if (KEY5 == key) {
                            g_ws = WS_GUI1;
                            intick = 0;
                        }
                        break;
                    case WS_GUI1:
                        if (KEY1 == key)
                            g_ws = WS_GUI4;
                        else if (KEY4 == key)
                            g_ws = WS_GUI2;
                        else if (KEY2 == key)  //左翻页
                            page_index = (page_index == 1) ? 3 : (page_index - 1);
                        else if (KEY3 == key)  //右翻页
                            page_index = (page_index == 3) ? 1 : (page_index + 1);
                        break;
                    case WS_GUI2:
                        if (KEY1 == key)
                            g_ws = WS_GUI1;
                        else if (KEY4 == key) {
                            g_ws = WS_GUI3;
                            Init_HC05();
                        } else if (KEY2 == key)  //左翻页
                            page_index = (page_index == 1) ? 5 : (page_index - 1);
                        else if (KEY3 == key)  //右翻页
                            page_index = (page_index == 5) ? 1 : (page_index + 1);
                        else if (KEY5 == key)  //控制采集数据时间间隔
                            mpu_interval = (mpu_interval == 50) ? 1000 : 50;
                        break;
                    case WS_GUI3:
                        if (KEY1 == key)
                            g_ws = WS_GUI2;
                        else if (KEY4 == key)
                            g_ws = WS_GUI4;
                        else if (KEY2 == key)
                            g_bUping = 1;
                        else if (KEY3 == key)
                            g_bUping = 0;
                        else if (KEY5 == key && 1 == page_index) {
                            if (1 == page_index) {
                                Init_HC05();
                            }
                        }
                        break;
                    case WS_GUI4:
                        if (KEY1 == key) {
                            g_ws = WS_GUI3;
                            Init_HC05();
                        } else if (KEY4 == key)
                            g_ws = WS_GUI1;
                        break;
                    case WS_START:
                        g_ws = WS_LOGO;
                        break;
                    default:
                        break;
                }
            }
        }
        if (mpuwarn || tempwarn) {
            if (Stop_Warn()) {  //停止报警
                mpuwarn = tempwarn = 0;
                for (int i = 0; i < 4; i++)
                    num[i] = ' ';
                warntick = 0;
            }
            DispSeg(num, 2);
        }
        osDelay(1);
    }
#pragma clang diagnostic pop
    /* USER CODE END StartKeyTask */
}

/* USER CODE BEGIN Header_StartUartTask */
/**
* @brief Function implementing the UartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartTask */
void StartUartTask(void *argument) {
    /* USER CODE BEGIN StartUartTask */
    StartRecvUart1();
    HC05_Init();
    /* Infinite loop */
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    for (;;) {
        if (recv1_len > 0) {
            printf("%s", recv1_buff);
            recv1_len = 0;
        }
        HC05_Proc();
        if (hc05.recv_len > 0) {
            printf("%s", hc05.recv_data);
            hc05.recv_len = 0;
        }
        osDelay(1);
    }
#pragma clang diagnostic pop
    /* USER CODE END StartUartTask */
}

/* USER CODE BEGIN Header_StartGUITask */
/**
* @brief GUI任务
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGUITask */
void StartGUITask(void *argument) {
    /* USER CODE BEGIN StartGUITask */
    GUI_Init();
    /* Infinite loop */
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    for (;;) {
        switch (g_ws) {
            case WS_LOGO:
                DrawLogo();
                WSLogo();
                break;
            case WS_GUI1:
                DrawGUI1();
                SetLeds(0);
                break;
            case WS_GUI2:
                DrawGUI2();
                break;
            case WS_GUI3:
                DrawGUI3();
                break;
            case WS_GUI4:
                DrawGUI4();
                break;
            default:
                break;
        }
        BeepDone();
        osDelay(1);

    }
#pragma clang diagnostic pop
    /* USER CODE END StartGUITask */
}

/* USER CODE BEGIN Header_StartGetDataTask */
/**
* @brief Function implementing the GetDataTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGetDataTask */
void StartGetDataTask(void *argument) {
    /* USER CODE BEGIN StartGetDataTask */
    /* Infinite loop */
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
    for (;;) {
        osDelay(1);
    }
#pragma clang diagnostic pop
    /* USER CODE END StartGetDataTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/*...WSLOGO和DrawLogo函数可合并*/
/**
 * @brief：启动界面
 */
void WSLogo(void) {
    static uint32_t tick = 0;
    static uint8_t leds_sta = 0x00;
    if (0 == intick)
        intick = osKernelGetTickCount();
    else {
        if (osKernelGetTickCount() >= intick + 7000) {
            intick = 0;
            g_ws = WS_GUI1;
            Beep(500, 3);
        }
    }
    SetLeds(leds_sta);
    if (osKernelGetTickCount() >= tick + 500) {
        leds_sta = ~leds_sta;
        tick = osKernelGetTickCount();
    }
}

void DrawLogo(void) {
    if (0 == intick)
        intick = osKernelGetTickCount();
    else {
        if (osKernelGetTickCount() <= intick + 2000) {
            GUI_Clear();
            GUI_SetFont(&GUI_FontHZ_SimSun_16);
            GUI_DispStringHCenterAt("专业实践\n综合设计II\n防火防盗监测器", 64, 6);
            GUI_Update();
        } else if (osKernelGetTickCount() <= intick + 5000) {
            GUI_Clear();
            GUI_SetFont(&GUI_FontHZ_SimSun_16);
            GUI_DispStringAt("成员1：郑雨欣", 0, 0);
            GUI_DispStringAt("18151119", 54, 16);
            GUI_DispStringAt("成员2：敖毅", 0, 32);
            GUI_DispStringAt("18042207", 54, 48);
            GUI_Update();
        } else if (osKernelGetTickCount() <= intick + 7000 && osKernelGetTickCount() > intick + 5000) {
            GUI_Clear();
            GUI_DrawBitmap(&bmzyx3, 0, 0);
            GUI_DrawBitmap(&bmaoyi1, 64, 0);
            GUI_Update();
        } else
            intick = 0;
    }
}

/*----------------------------------主菜单--------------------------------*/
void DrawGUI1(void) {
    char buf[20];
    struct SplitFloat float_data;
    GUI_Clear();
    GUI_SetFont(&GUI_FontHZ_SimSun_12);
    GUI_SetColor(GUI_COLOR_BLACK);
    GUI_DispStringAt("实时监测", 0, 0);
    GUI_SetColor(GUI_COLOR_WHITE);
    GUI_DispStringAt("数据曲线", 0, 13);
    GUI_DispStringAt("无线通信", 0, 26);
    GUI_DispStringAt("参数设置", 0, 39);
    GUI_DispStringAt("K1︽ K2《 K3》 K4︾", 0, 52);
    GUI_DrawHLine(52, 0, 128);
    GUI_DrawVLine(48, 0, 52);
    switch (page_index) {
        case 1:
            GUI_DispStringAt("当前温度:", 50, 0);
            GUI_DispStringAt("震动报警:", 50, 26);
            float_data = split_float(temp);  //温度
            sprintf(buf, "%d.%d℃", float_data.integer, float_data.decimal);
            GUI_DispStringAt(buf, 90, 13);
            GUI_DispStringAt(mpuwarn ? "是" : "否", 90, 39);
            break;
        case 2:
            sprintf(buf, "ax:%6d", ax);
            GUI_DispStringAt(buf, 50, 2);
            sprintf(buf, "ay:%6d", ay);
            GUI_DispStringAt(buf, 50, 20);
            sprintf(buf, "az:%6d", az);
            GUI_DispStringAt(buf, 50, 38);
            break;
        case 3:
            sprintf(buf, "gx:%6d", gx);
            GUI_DispStringAt(buf, 50, 2);
            sprintf(buf, "gy:%6d", gy);
            GUI_DispStringAt(buf, 50, 20);
            sprintf(buf, "gz:%6d", gz);
            GUI_DispStringAt(buf, 50, 38);
            break;
        default:
            break;
    }
    GUI_Update();
}

void DrawGUI2(void) {
    char buf[20];
    static float t_low = 0;
    struct SplitFloat float_data;
    t_low = T_decimal / 10.0;
    int i;
    GUI_Clear();
    GUI_SetFont(&GUI_FontHZ_SimSun_12);
    GUI_DispStringAt("实时监测", 0, 0);
    GUI_SetColor(GUI_COLOR_BLACK);
    GUI_DispStringAt("数据曲线", 0, 13);
    GUI_SetColor(GUI_COLOR_WHITE);
    GUI_DispStringAt("无线通信", 0, 26);
    GUI_DispStringAt("参数设置", 0, 39);
    GUI_DispStringAt("K1︽ K2《 K3》 K4︾", 0, 52);
    GUI_DrawHLine(52, 0, 128);
    GUI_DrawVLine(48, 0, 52);
    int sw = 128 - 48;
    int sh = 64 - 12 - 12;
    int ox = 48;
    int oy = 12 + sh;
    switch (page_index) {
        case 1: {
            float tempMin = 25;
            float tempMax = 35;
            float tempLmt = 30;
            float dh = sh / (tempMax - tempMin);
            float_data = split_float(temp);  //温度
            sprintf(buf, "温度：%d.%d℃", float_data.integer, float_data.decimal);
            GUI_DispStringAt(buf, 50, 0);
            for (i = 0; i < MAX_DATALEN; i += 6) {
                GUI_DrawHLine(oy - (tempLmt - tempMin) * dh, ox + i, ox + i + 3);
            }
            for (i = 0; i < cTemp && i < MAX_DATALEN; ++i) {
                GUI_DrawLine(ox + i - 1, oy - (vTemp[i - 1] - tempMin) * dh, ox + i, oy - (vTemp[i] - tempMin) * dh);
            }
        }
            break;
        case 2: {
            float tempMin = -90;
            float tempMax = 90;
            float dh = sh / (tempMax - tempMin);
            float_data = split_float(fAX);  //俯仰角
            sprintf(buf, "俯仰角：%d.%d°", float_data.integer, float_data.decimal);
            GUI_DispStringAt(buf, 50, 0);
            for (i = 0; i < MAX_DATALEN; i += 6) {
                GUI_DrawHLine(oy + tempMin * dh, ox + i, ox + sw);
            }
            for (i = 0; i < cPitch && i < MAX_DATALEN; ++i) {
                GUI_DrawLine(ox + i - 1, oy - (vPitch[i - 1] - tempMin) * dh, ox + i, oy - (vPitch[i] - tempMin) * dh);
            }
        }
            break;
        case 3: {
            float tempMin = -90;
            float tempMax = 90;
            float dh = sh / (tempMax - tempMin);
            float_data = split_float(fAY);  //横滚角
            sprintf(buf, "横滚角：%d.%d°", float_data.integer, float_data.decimal);
            GUI_DispStringAt(buf, 50, 0);
            for (i = 0; i < MAX_DATALEN; i += 6) {
                GUI_DrawHLine(oy + tempMin * dh, ox + i, ox + sw);
            }
            for (i = 0; i < cRoll && i < MAX_DATALEN; ++i) {
                GUI_DrawLine(ox + i - 1, oy - (vRoll[i - 1] - tempMin) * dh, ox + i, oy - (vRoll[i] - tempMin) * dh);
            }
        }
            break;
        case 4: {
            float tempMin = -180;
            float tempMax = 180;
            float dh = sh / (tempMax - tempMin);
            float_data = split_float(fAZ);  //航向角
            sprintf(buf, "航向角：%d.%d°", float_data.integer, float_data.decimal);
            GUI_DispStringAt(buf, 50, 0);
            for (i = 0; i < MAX_DATALEN; i += 6) {
                GUI_DrawHLine(oy + tempMin * dh, ox + i, ox + sw);
            }
            for (i = 0; i < cYaw && i < MAX_DATALEN; ++i) {
                GUI_DrawLine(ox + i - 1, oy - (vYaw[i - 1] - tempMin) * dh, ox + i, oy - (vYaw[i] - tempMin) * dh);
            }
        }
            break;
        case 5: {
            ox = (48 + 128) / 2;
            oy = (12 + 40) / 2 + 2;
            RateCube(fAX, fAY, fAZ, GUI_COLOR_WHITE, ox, oy);
            RotatePic32X32(TAB_zheng, fAX, fAY, fAZ, GUI_COLOR_WHITE, ox - 16, oy - 16, 16);
        }
            break;
        default:
            break;
    }
    GUI_Update();
}

void DrawGUI3(void) {
    GUI_Clear();
    GUI_SetFont(&GUI_FontHZ_SimSun_12);
    GUI_DispStringAt("实时监测", 0, 0);
    GUI_DispStringAt("数据曲线", 0, 13);
    GUI_SetColor(GUI_COLOR_BLACK);
    GUI_DispStringAt("无线通信", 0, 26);
    GUI_SetColor(GUI_COLOR_WHITE);
    GUI_DispStringAt("参数设置", 0, 39);
    GUI_DispStringAt("K1︽ K2《 K3》 K4︾", 0, 52);
    GUI_DrawHLine(52, 0, 128);
    GUI_DrawVLine(48, 0, 52);
    GUI_DispStringAt((char *) hc05.name, 50, 0);
    GUI_DispStringAt(HC05_IsConn() ? "已连接" : "未连接", 50, 16);
    GUI_DispStringAt(g_bUping ? "上传中" : "未上传", 50, 32);
    GUI_Update();
}

void DrawGUI4(void) {
    GUI_Clear();
    GUI_SetFont(&GUI_FontHZ_SimSun_12);
    GUI_DispStringAt("实时监测", 0, 0);
    GUI_DispStringAt("数据曲线", 0, 13);
    GUI_DispStringAt("无线通信", 0, 26);
    GUI_SetColor(GUI_COLOR_BLACK);
    GUI_DispStringAt("参数设置", 0, 39);
    GUI_SetColor(GUI_COLOR_WHITE);
    GUI_DispStringAt("K1︽ K2《 K3》 K4︾", 0, 52);
    GUI_DrawHLine(52, 0, 128);
    GUI_DrawVLine(48, 0, 52);
    GUI_Update();
}

void ShowLines() {
    char str[30];
    GUI_Clear();
    GUI_DrawHLine(49, 0, 128);
    GUI_DrawVLine(49, 0, 49);

    GUI_SetFont(&GUI_FontHZ_SimSun_12);
    GUI_DispStringAt("动态曲线", 1, 24);

    GUI_SetColor(GUI_COLOR_BLACK);
    GUI_DispStringAt("系统测试", 1, 0);
    GUI_DispStringAt("调试灯", 1, 12);
    GUI_DispStringAt("无线通信", 1, 36);
    GUI_SetColor(GUI_COLOR_WHITE);
    GUI_DispStringAt("K1︽ K2《 K3》 K4︾", 0, 52);
}

struct SplitFloat split_float(float data) {
    struct SplitFloat int_decimal;
    float data1 = fabs(data);
    int_decimal.integer = (int) data1;
    float low = (data1 - int_decimal.integer) * 10;
    int_decimal.decimal = (int) (low + 0.5) > (int) low ? (int) low + 1 : (int) low;
    int_decimal.integer = (data > 0) ? int_decimal.integer : int_decimal.integer * (-1);
    return int_decimal;
}

/**
 * @brief 蜂鸣器使能
 * @param time 鸣叫时间
 * @param tune 声调（7阶）
 */
void Beep(int time, int tune) {
    static uint16_t TAB[] = {494, 523, 588, 660, 698, 784, 880, 988};
    HAL_TIM_Base_Start(&htim3);
    if (tune >= 1 && tune <= 7) {
        int pre = 1000000 / TAB[tune];
        __HAL_TIM_SET_AUTORELOAD(&htim3, pre);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pre / 2);
        beeptick = osKernelGetTickCount() + time;
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    }
    while (time-- > 0) {
        HAL_GPIO_TogglePin(BEEP_GPIO_Port, BEEP_Pin);
        osDelay(1);
    }
}

/**
 * @brief:蜂鸣器鸣叫结束
 */
void BeepDone(void) {
    if (beeptick > 0 && osKernelGetTickCount() >= beeptick) {
        beeptick = 0;
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    }
}

/**
 * @brief 数码管扫描
 * @param num 4位数组
 * @param dot 小数点位置
 */
void DispSeg(uint8_t num[4], uint8_t dot) {
    for (int i = 0; i < 4; i++) {
        Write595(i, num[i], (dot == (i + 1) ? 1 : 0));
        osDelay(1);
    }
}

void Init_HC05(void) {
    if (HC05_IsOK()) {
        HC05_SetRole(0);
        HC05_GetName();
        printf("HC05 Name:%s\n", hc05.name);
        HC05_AtMode(0);
    }
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
