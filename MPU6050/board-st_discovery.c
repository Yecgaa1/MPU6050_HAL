#include "stm32f1xx.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "board-st_discovery.h"

#include "stdio.h"
#include "main.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
#include "packet.h"
/* Private typedef -----------------------------------------------------------*/
/*从MPL读取的数据。 */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define PRINT_COMPASS   (0x08)
#define PRINT_EULER     (0x10)
#define PRINT_ROT_MAT   (0x20)
#define PRINT_HEADING   (0x40)
#define PRINT_PEDO      (0x80)
#define PRINT_LINEAR_ACCEL (0x100)
#define PRINT_GRAVITY_VECTOR (0x200)

volatile uint32_t hal_timestamp = 0;
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define COMPASS_ON      (0x04)

#define MOTION          (0)
#define NO_MOTION       (1)

/*开始采样率。 */
#define DEFAULT_MPU_HZ  (20)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)

#define PEDO_READ_MS    (1000)
#define TEMP_READ_MS    (500)
#define COMPASS_READ_MS (100)
struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
    unsigned int report;
    unsigned short dmp_features;
    struct rx_s rx;
};
static struct hal_s hal = {0};

/*USB RX二进制信号量。实际上，这只是一个标志。不包含在结构中
 *，因为在其他地方已声明为extern。
 */
volatile unsigned char rx_new;

unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";

/*特定于平台的信息。有点像董事会文件。 */
struct platform_data_s {
    signed char orientation[9];
};

/*传感器可以任何方向安装到板上。安装
 *下方显示的矩阵告诉MPL如何旋转来自
 *驱动程序。
 *TODO：以下矩阵是指内部测试中的配置
 *Invensense的董事会。如果需要，请修改矩阵以匹配
 *针对特定设置的芯片到矩阵矩阵。
 */
static struct platform_data_s gyro_pdata = {
        .orientation = { 1, 0, 0,
                         0, 1, 0,
                         0, 0, 1}
};

#if defined MPU9150 || defined MPU9250
static struct platform_data_s compass_pdata = {
    .orientation = { 0, 1, 0,
                     1, 0, 0,
                     0, 0, -1}
};
#define COMPASS_ENABLED 1
#elif defined AK8975_SECONDARY
static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                     0, 1, 0,
                     0, 0,-1}
};
#define COMPASS_ENABLED 1
#elif defined AK8963_SECONDARY
static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                     0,-1, 0,
                     0, 0, 1}
};
#define COMPASS_ENABLED 1
#endif


/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* ---------------------------------------------------------------------------*/
/*从MPL获取数据。
 *TODO：将返回值添加到inv_get_sensor_type_xxx API中以进行区分
 *在新数据和陈旧数据之间。
 */
static void read_from_mpl(void)
{
    long msg, data[9];
    int8_t accuracy;
    unsigned long timestamp;
    float float_data[3] = {0};
    if (inv_get_sensor_type_euler(data, &accuracy,
                                  (inv_time_t*)&timestamp))
    {
        float Pitch,Roll,Yaw;
        //inv_get_sensor_type_euler读出的数据是Q16格式，所以左移16位.
        Pitch =data[0]*1.0/(1<<16) ;
        Roll = data[1]*1.0/(1<<16);
        Yaw = data[2]*1.0/(1<<16);
        printf("Pitch :  %.4f  ",Pitch);
        printf("Roll  :  %.4f  ", Roll );
        printf("Yaw   :  %.4f  ", Yaw );
        /*温度*/
        mpu_get_temperature(data,(inv_time_t*)&timestamp);
        printf("Temperature  :  %.2f  \r\n", data[0]*1.0/(1<<16) );
    }
    get_tick_count(&timestamp);
    if (timestamp > hal.next_pedo_ms)
    {
        hal.next_pedo_ms = timestamp + PEDO_READ_MS;
        unsigned long step_count, walk_time;
        dmp_get_pedometer_step_count(&step_count);
        dmp_get_pedometer_walk_time(&walk_time);
        printf("Walked %ld steps over %ld milliseconds..\n", step_count,
               walk_time);
    }

//    if (inv_get_sensor_type_quat(data, &accuracy, (inv_time_t*)&timestamp)) {
//       /*将四元数数据包发送到PC。由于这是Python使用的
//        *测试应用程序以可视方式表示3D四元数，每次发送一次
//        *MPL有新数据。
//        */
//        eMPL_send_quat(data);

//        /*可以使用USB命令发送或禁止特定的数据包。 */
//        if (hal.report & PRINT_QUAT)
//            eMPL_send_data(PACKET_DATA_QUAT, data);
//    }

//    if (hal.report & PRINT_ACCEL) {
//        if (inv_get_sensor_type_accel(data, &accuracy,
//            (inv_time_t*)&timestamp))
//            eMPL_send_data(PACKET_DATA_ACCEL, data);
//    }
//    if (hal.report & PRINT_GYRO) {
//        if (inv_get_sensor_type_gyro(data, &accuracy,
//            (inv_time_t*)&timestamp))
//            eMPL_send_data(PACKET_DATA_GYRO, data);
//    }
//#ifdef COMPASS_ENABLED
//    if (hal.report & PRINT_COMPASS) {
//        if (inv_get_sensor_type_compass(data, &accuracy,
//            (inv_time_t*)&timestamp))
//            eMPL_send_data(PACKET_DATA_COMPASS, data);
//    }
//#endif
//    if (hal.report & PRINT_EULER) {
//        if (inv_get_sensor_type_euler(data, &accuracy,
//            (inv_time_t*)&timestamp))
//            eMPL_send_data(PACKET_DATA_EULER, data);
//    }
//    if (hal.report & PRINT_ROT_MAT) {
//        if (inv_get_sensor_type_rot_mat(data, &accuracy,
//            (inv_time_t*)&timestamp))
//            eMPL_send_data(PACKET_DATA_ROT, data);
//    }
//    if (hal.report & PRINT_HEADING) {
//        if (inv_get_sensor_type_heading(data, &accuracy,
//            (inv_time_t*)&timestamp))
//            eMPL_send_data(PACKET_DATA_HEADING, data);
//    }
//    if (hal.report & PRINT_LINEAR_ACCEL) {
//        if (inv_get_sensor_type_linear_acceleration(float_data, &accuracy, (inv_time_t*)&timestamp)) {
//        	MPL_LOGI("Linear Accel: %7.5f %7.5f %7.5f\r\n",
//        			float_data[0], float_data[1], float_data[2]);
//         }
//    }
//    if (hal.report & PRINT_GRAVITY_VECTOR) {
//            if (inv_get_sensor_type_gravity(float_data, &accuracy,
//                (inv_time_t*)&timestamp))
//            	MPL_LOGI("Gravity Vector: %7.5f %7.5f %7.5f\r\n",
//            			float_data[0], float_data[1], float_data[2]);
//    }
//    if (hal.report & PRINT_PEDO) {
//        unsigned long timestamp;
//        get_tick_count(&timestamp);
//        if (timestamp > hal.next_pedo_ms) {
//            hal.next_pedo_ms = timestamp + PEDO_READ_MS;
//            unsigned long step_count, walk_time;
//            dmp_get_pedometer_step_count(&step_count);
//            dmp_get_pedometer_walk_time(&walk_time);
//            MPL_LOGI("Walked %ld steps over %ld milliseconds..\n", step_count,
//            walk_time);
//        }
//    }

//    /*只要MPL检测到运动状态发生变化，应用程序就可以
//     *被通知。在本例中，我们使用一个LED代表当前
//     *运动状态。
//     */
//    msg = inv_get_message_level_0(INV_MSG_MOTION_EVENT |
//            INV_MSG_NO_MOTION_EVENT);
//    if (msg) {
//        if (msg & INV_MSG_MOTION_EVENT) {
//            MPL_LOGI("Motion!\n");
//        } else if (msg & INV_MSG_NO_MOTION_EVENT) {
//            MPL_LOGI("No motion!\n");
//        }
//    }
}

#ifdef COMPASS_ENABLED
void send_status_compass() {
	long data[3] = { 0 };
	int8_t accuracy = { 0 };
	unsigned long timestamp;
	inv_get_compass_set(data, &accuracy, (inv_time_t*) &timestamp);
	MPL_LOGI("Compass: %7.4f %7.4f %7.4f ",
			data[0]/65536.f, data[1]/65536.f, data[2]/65536.f);
	MPL_LOGI("Accuracy= %d\r\n", accuracy);

}
#endif

/*处理传感器的开/关组合。 */
static void setup_gyro(void)
{
    unsigned char mask = 0, lp_accel_was_on = 0;
    if (hal.sensors & ACCEL_ON)
        mask |= INV_XYZ_ACCEL;
    if (hal.sensors & GYRO_ON) {
        mask |= INV_XYZ_GYRO;
        lp_accel_was_on |= hal.lp_accel_mode;
    }
#ifdef COMPASS_ENABLED
    if (hal.sensors & COMPASS_ON) {
        mask |= INV_XYZ_COMPASS;
        lp_accel_was_on |= hal.lp_accel_mode;
    }
#endif
    /*如果需要电源转换，则应使用
     *传感器的遮罩仍处于启用状态。驱动程序关闭所有传感器
     *排除在这个面具之外。
     */
    mpu_set_sensors(mask);
    mpu_configure_fifo(mask);
    if (lp_accel_was_on) {
        unsigned short rate;
        hal.lp_accel_mode = 0;
        /*退出LP加速度，将新的加速度采样率通知MPL。 */
        mpu_get_sample_rate(&rate);
        inv_set_accel_sample_rate(1000000L / rate);
    }
}

static void tap_cb(unsigned char direction, unsigned char count)
{
    switch (direction) {
        case TAP_X_UP:
            MPL_LOGI("Tap X+ ");
            break;
        case TAP_X_DOWN:
            MPL_LOGI("Tap X- ");
            break;
        case TAP_Y_UP:
            MPL_LOGI("Tap Y+ ");
            break;
        case TAP_Y_DOWN:
            MPL_LOGI("Tap Y- ");
            break;
        case TAP_Z_UP:
            MPL_LOGI("Tap Z+ ");
            break;
        case TAP_Z_DOWN:
            MPL_LOGI("Tap Z- ");
            break;
        default:
            return;
    }
    MPL_LOGI("x%d\n", count);
    return;
}

static void android_orient_cb(unsigned char orientation)
{
    switch (orientation) {
        case ANDROID_ORIENT_PORTRAIT:
            MPL_LOGI("Portrait\n");
            break;
        case ANDROID_ORIENT_LANDSCAPE:
            MPL_LOGI("Landscape\n");
            break;
        case ANDROID_ORIENT_REVERSE_PORTRAIT:
            MPL_LOGI("Reverse Portrait\n");
            break;
        case ANDROID_ORIENT_REVERSE_LANDSCAPE:
            MPL_LOGI("Reverse Landscape\n");
            break;
        default:
            return;
    }
}


static inline void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

#if defined (MPU6500) || defined (MPU9250)
    result = mpu_run_6500_self_test(gyro, accel, 0);
#elif defined (MPU6050) || defined (MPU9150)
    result = mpu_run_self_test(gyro, accel);
#endif
    if (result == 0x7) {
        MPL_LOGI("Passed!\n");
        MPL_LOGI("accel: %7.4f %7.4f %7.4f\n",
                 accel[0]/65536.f,
                 accel[1]/65536.f,
                 accel[2]/65536.f);
        MPL_LOGI("gyro: %7.4f %7.4f %7.4f\n",
                 gyro[0]/65536.f,
                 gyro[1]/65536.f,
                 gyro[2]/65536.f);
        /*通过测试。我们可以在这里信任陀螺仪数据，因此现在我们需要更新校准数据*/

#ifdef USE_CAL_HW_REGISTERS
        /*
         *这部分代码使用MPUxxxx器件中的HW偏移寄存器
         *而不是将校准数据推送到MPL软件库
         */
        unsigned char i = 0;

        for(i = 0; i<3; i++) {
        	gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
        	accel[i] *= 2048.f; //convert to +-16G
        	accel[i] = accel[i] >> 16;
        	gyro[i] = (long)(gyro[i] >> 16);
        }

        mpu_set_gyro_bias_reg(gyro);

#if defined (MPU6500) || defined (MPU9250)
        mpu_set_accel_bias_6500_reg(accel);
#elif defined (MPU6050) || defined (MPU9150)
        mpu_set_accel_bias_6050_reg(accel);
#endif
#else
        /*将校准后的数据推入MPL库。
         *
         *MPL预计硬件单元<< 16会有偏差，但自检会返回
        *g的<< 16。
        */
        unsigned short accel_sens;
        float gyro_sens;

        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        inv_set_accel_bias(accel, 3);
        mpu_get_gyro_sens(&gyro_sens);
        gyro[0] = (long) (gyro[0] * gyro_sens);
        gyro[1] = (long) (gyro[1] * gyro_sens);
        gyro[2] = (long) (gyro[2] * gyro_sens);
        inv_set_gyro_bias(gyro, 3);
#endif
    }
    else {
        if (!(result & 0x1))
            MPL_LOGE("Gyro failed.\n");
        if (!(result & 0x2))
            MPL_LOGE("Accel failed.\n");
        if (!(result & 0x4))
            MPL_LOGE("Compass failed.\n");
    }

}

//static void handle_input(void)
//{
//
//    char c = USART_ReceiveData(USART2);

//    switch (c) {
//    /*这些命令关闭单个传感器。 */
//    case '8':
//        hal.sensors ^= ACCEL_ON;
//        setup_gyro();
//        if (!(hal.sensors & ACCEL_ON))
//            inv_accel_was_turned_off();
//        break;
//    case '9':
//        hal.sensors ^= GYRO_ON;
//        setup_gyro();
//        if (!(hal.sensors & GYRO_ON))
//            inv_gyro_was_turned_off();
//        break;
//#ifdef COMPASS_ENABLED
//    case '0':
//        hal.sensors ^= COMPASS_ON;
//        setup_gyro();
//        if (!(hal.sensors & COMPASS_ON))
//            inv_compass_was_turned_off();
//        break;
//#endif
//    /*这些命令将单个传感器数据或融合数据发送到PC。 */
//    case 'a':
//        hal.report ^= PRINT_ACCEL;
//        break;
//    case 'g':
//        hal.report ^= PRINT_GYRO;
//        break;
//#ifdef COMPASS_ENABLED
//    case 'c':
//        hal.report ^= PRINT_COMPASS;
//        break;
//#endif
//    case 'e':
//        hal.report ^= PRINT_EULER;
//        break;
//    case 'r':
//        hal.report ^= PRINT_ROT_MAT;
//        break;
//    case 'q':
//        hal.report ^= PRINT_QUAT;
//        break;
//    case 'h':
//        hal.report ^= PRINT_HEADING;
//        break;
//    case 'i':
//        hal.report ^= PRINT_LINEAR_ACCEL;
//        break;
//    case 'o':
//        hal.report ^= PRINT_GRAVITY_VECTOR;
//        break;
//#ifdef COMPASS_ENABLED
//	case 'w':
//		send_status_compass();
//		break;
//#endif
//    /*此命令打印出每个陀螺寄存器的值以进行调试。
//     *如果禁用日志记录，此功能无效。
//     */
//    case 'd':
//        mpu_reg_dump();
//        break;
//    /*测试低功耗加速模式。 */
//    case 'p':
//        if (hal.dmp_on)
//            /*LP加速与DMP不兼容。 */
//            break;
//        mpu_lp_accel_mode(20);
//        /*启用LP加速模式时，驱动程序将自动配置
//         *锁存中断的硬件。但是，有时MCU
//         *错过上升/下降沿，并且hal.new_gyro标志永远不会
//         *设置。为了避免锁定在这种状态下，我们覆盖了
//         *驱动程序的配置并坚持未锁定的中断模式。
//         *
//         *TODO：MCU支持电平触发的中断。
//         */
//        mpu_set_int_latched(0);
//        hal.sensors &= ~(GYRO_ON|COMPASS_ON);
//        hal.sensors |= ACCEL_ON;
//        hal.lp_accel_mode = 1;
//        inv_gyro_was_turned_off();
//        inv_compass_was_turned_off();
//        break;
//    /*可以运行硬件自检，而无需与
//     *MPL，因为它完全位于陀螺驱动程序中。正在记录
//     *假定已启用；否则，可能会使用几个LED
//     *在此处显示测试结果。
//     */
//    case 't':
//        run_self_test();
//        /*让MPL知道连续性已中断。 */
//        inv_accel_was_turned_off();
//        inv_gyro_was_turned_off();
//        inv_compass_was_turned_off();
//        break;
//    /*取决于您的应用程序，可能需要更快或更高的传感器数据
//     *速度较慢。这些命令可以加快或减慢
//     *传感器数据被推送到MPL。
//     *
//     *在此示例中，指南针速率从未改变。
//     */
//    case '1':
//        if (hal.dmp_on) {
//            dmp_set_fifo_rate(10);
//            inv_set_quat_sample_rate(100000L);
//        } else
//            mpu_set_sample_rate(10);
//        inv_set_gyro_sample_rate(100000L);
//        inv_set_accel_sample_rate(100000L);
//        break;
//    case '2':
//        if (hal.dmp_on) {
//            dmp_set_fifo_rate(20);
//            inv_set_quat_sample_rate(50000L);
//        } else
//            mpu_set_sample_rate(20);
//        inv_set_gyro_sample_rate(50000L);
//        inv_set_accel_sample_rate(50000L);
//        break;
//    case '3':
//        if (hal.dmp_on) {
//            dmp_set_fifo_rate(40);
//            inv_set_quat_sample_rate(25000L);
//        } else
//            mpu_set_sample_rate(40);
//        inv_set_gyro_sample_rate(25000L);
//        inv_set_accel_sample_rate(25000L);
//        break;
//    case '4':
//        if (hal.dmp_on) {
//            dmp_set_fifo_rate(50);
//            inv_set_quat_sample_rate(20000L);
//        } else
//            mpu_set_sample_rate(50);
//        inv_set_gyro_sample_rate(20000L);
//        inv_set_accel_sample_rate(20000L);
//        break;
//    case '5':
//        if (hal.dmp_on) {
//            dmp_set_fifo_rate(100);
//            inv_set_quat_sample_rate(10000L);
//        } else
//            mpu_set_sample_rate(100);
//        inv_set_gyro_sample_rate(10000L);
//        inv_set_accel_sample_rate(10000L);
//        break;
//	case ',':
//        /*将硬件设置为仅在手势事件时中断。此功能是
//         *有助于使MCU保持睡眠状态，直到DMP检测到敲击或
//         *方向事件。
//         */
//        dmp_set_interrupt_mode(DMP_INT_GESTURE);
//        break;
//    case '.':
//        /*将硬件设置为定期中断。 */
//        dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);
//        break;
//    case '6':
//        /* Toggle pedometer display. */
//        hal.report ^= PRINT_PEDO;
//        break;
//    case '7':
//        /* Reset pedometer. */
//        dmp_set_pedometer_step_count(0);
//        dmp_set_pedometer_walk_time(0);
//        break;
//    case 'f':
//        if (hal.lp_accel_mode)
//            /*LP加速与DMP不兼容。 */
//            return;
//        /* 切换 DMP. */
//        if (hal.dmp_on) {
//            unsigned short dmp_rate;
//            unsigned char mask = 0;
//            hal.dmp_on = 0;
//            mpu_set_dmp_state(0);
//            /* 恢复FIFO设置. */
//            if (hal.sensors & ACCEL_ON)
//                mask |= INV_XYZ_ACCEL;
//            if (hal.sensors & GYRO_ON)
//                mask |= INV_XYZ_GYRO;
//            if (hal.sensors & COMPASS_ON)
//                mask |= INV_XYZ_COMPASS;
//            mpu_configure_fifo(mask);
//            /*当使用DMP时，硬件采样率固定在
//             *200hz，并且DMP被配置为使用函数dmp_set_fifo_rate对FIFO输出
//             *进行下采样。 然而，当DMP关闭时，采样率保持在200Hz。 这可以在inv_mpu中处理
//             *，但它需要知道*inv_mpu_dmp_motion_driver.c存在。
//             *为了避免这一点，我们将只将额外的逻辑放在应用层。
//             */
//            dmp_get_fifo_rate(&dmp_rate);
//            mpu_set_sample_rate(dmp_rate);
//            inv_quaternion_sensor_was_turned_off();
//            MPL_LOGI("DMP disabled.\n");
//        } else {
//            unsigned short sample_rate;
//            hal.dmp_on = 1;
//            /* 保持当前FIFO速率
//             */
//            mpu_get_sample_rate(&sample_rate);
//            dmp_set_fifo_rate(sample_rate);
//            inv_set_quat_sample_rate(1000000L / sample_rate);
//            mpu_set_dmp_state(1);
//            MPL_LOGI("DMP enabled.\n");
//        }
//        break;
//    case 'm':
//        /*测试运动中断硬件功能。 */
//	#ifndef MPU6050 // not enabled for 6050 product
//	hal.motion_int_mode = 1;
//	#endif
//        break;

//    case 'v':
//        /*切换LP四元数。
//          * DMP功能可以在运行时启用/禁用。 使用相同的
//          *其他功能的处理方法。
//          */
//        hal.dmp_features ^= DMP_FEATURE_6X_LP_QUAT;
//        dmp_enable_feature(hal.dmp_features);
//        if (!(hal.dmp_features & DMP_FEATURE_6X_LP_QUAT)) {
//            inv_quaternion_sensor_was_turned_off();
//            MPL_LOGI("LP quaternion disabled.\n");
//        } else
//            MPL_LOGI("LP quaternion enabled.\n");
//        break;
//    default:
//        break;
//    }
//    hal.rx.cmd = 0;
//}

/*每当有新的陀螺仪数据可用时，都会在
  * ISR上下文。 在此示例中，它设置一个标志来保护FIFO读取
  *功能。
  */
void gyro_data_ready_cb(void)
{
    hal.new_gyro = 1;
}
/*******************************************************************************/

/**
   * @简短的主要入口点。
   * @par参数无
   * @retval无效无
   * @par必需的前提条件：无
   */

void mpu_test(void)
{
    inv_error_t result;
    unsigned char accel_fsr,  new_temp = 0;
    unsigned short gyro_rate, gyro_fsr;
    unsigned long timestamp;
    struct int_param_s int_param;

#ifdef COMPASS_ENABLED
    unsigned char new_compass = 0;
		unsigned short compass_fsr;
#endif


    result = mpu_init(&int_param);
    if (result)
    {
        MPL_LOGE("Could not initialize gyro.\r\n");
    }


    /*如果您未使用MPU9150并且未使用DMP功能，则此
    *该功能会将所有从机置于主总线上。
    * mpu_set_bypass（1）;
    */

    result = inv_init_mpl();
    if (result)
    {
        MPL_LOGE("Could not initialize MPL.\r\n");
    }

    /*计算6轴和9轴四元数。 */
    inv_enable_quaternion();
    inv_enable_9x_sensor_fusion();
    /* MPL期望指南针数据处于恒定速率（与速率匹配）
    *传递给inv_set_compass_sample_rate）。 如果这是您的问题
    *应用程序，调用此函数，MPL将取决于
    *时间戳传递给inv_build_compass。
    *
    * inv_9x_fusion_use_timestamps（1）;
    */

    /*不推荐使用此功能。
    * inv_enable_no_gyro_fusion（）;
    */

    /*在不运动时更新陀螺仪偏置。
    *警告：这些算法是互斥的。
    */
    inv_enable_fast_nomot();
    /* inv_enable_motion_no_motion(); */
    /* inv_set_no_motion_time(1000); */

    /*温度变化时更新陀螺仪偏置。 */
    inv_enable_gyro_tc();

    /*此算法在运动时更新加速度偏差。 更准确
    *进行自检时可以进行偏差测量（请参见
    * handle_input），但是如果自检失败，则可以启用此算法
    *在您的应用程序中执行。
    *
    * inv_enable_in_use_auto_calibration（）;
    */
#ifdef COMPASS_ENABLED
    /* Compass calibration algorithms. */
		inv_enable_vector_compass_cal();
		inv_enable_magnetic_disturbance();
#endif
    /*如果您需要在校准指南针之前估算航向，
    *启用此算法。 一个好的数字八之后，它就变得无用了
    *已检测到，因此我们将其省略以节省内存。
    * inv_enable_heading_from_gyro（）;
    */

    /*允许在read_from_mpl中使用MPL API。 */
    inv_enable_eMPL_outputs();

    result = inv_start_mpl();
    if (result == INV_ERROR_NOT_AUTHORIZED)
    {
        while (1)
        {
            MPL_LOGE("Not authorized.\n");
        }
    }
    if (result)
    {
        MPL_LOGE("Could not start the MPL.\n");
    }

    /*获取/设置硬件配置。 启动陀螺仪。 */
    /*唤醒所有传感器。 */
#ifdef COMPASS_ENABLED
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
#else
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
#endif
    /*将陀螺仪和加速数据都推入FIFO。 */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
#ifdef COMPASS_ENABLED
    /*罗盘采样率可以小于陀螺/加速度采样率。
		*使用此功能进行正确的电源管理。
		*/
		mpu_set_compass_sample_rate(1000 / COMPASS_READ_MS);
#endif
    /*如果配置不正确，请回读配置。*/
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);
#ifdef COMPASS_ENABLED
    mpu_get_compass_fsr(&compass_fsr);
#endif
    /*与MPL同步驱动程序配置。 */
    /*预期的采样率，以微秒为单位。 */
    inv_set_gyro_sample_rate(1000000L / gyro_rate);
    inv_set_accel_sample_rate(1000000L / gyro_rate);
#ifdef COMPASS_ENABLED
    /*指南针速率独立于陀螺仪和加速仪速率。 只要
		*以正确的值（9轴）调用inv_set_compass_sample_rate
		*融合算法的罗盘校正增益将正常工作。
		*/
		inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);
#endif
    /*设置芯片到身体的方向矩阵。
    *将硬件单位设置为dps / g /度的比例因子。
    */
    inv_set_gyro_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)gyro_fsr<<15);
    inv_set_accel_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)accel_fsr<<15);
#ifdef COMPASS_ENABLED
    inv_set_compass_orientation_and_scale(
						inv_orientation_matrix_to_scalar(compass_pdata.orientation),
						(long)compass_fsr<<15);
#endif
    /*初始化HAL状态变量。 */
#ifdef COMPASS_ENABLED
    hal.sensors = ACCEL_ON | GYRO_ON | COMPASS_ON;
#else
    hal.sensors = ACCEL_ON | GYRO_ON;
#endif
    hal.dmp_on = 0;
    hal.report = 0;
    hal.rx.cmd = 0;
    hal.next_pedo_ms = 0;
    hal.next_compass_ms = 0;
    hal.next_temp_ms = 0;

    /*指南针读取由调度程序处理。 */
    get_tick_count(&timestamp);

    /*要初始化DMP：
    * 1.调用dmp_load_motion_driver_firmware（）。这会将DMP映像推入
    * inv_mpu_dmp_motion_driver.h到MPU内存中。
    * 2.将陀螺和加速度定向矩阵推入DMP。
    * 3.注册手势回调。不用担心，这些回调不会
    *执行，除非启用了相应的功能。
    * 4.调用dmp_enable_feature（mask）以启用不同的功能。
    * 5.调用dmp_set_fifo_rate（freq）选择DMP输出速率。
    * 6.调用任何特定于功能的控制功能。
    *
    *要启用DMP，只需调用mpu_set_dmp_state（1）。该功能可以
    *反复调用以在运行时启用和禁用DMP。
    *
    *以下是DMP支持的功能的简短摘要
    * inv_mpu_dmp_motion_driver.c中提供的图像：
    * DMP_FEATURE_LP_QUAT：在DMP上生成仅陀螺四元数
    * 200Hz。以较高的速率集成陀螺仪数据可减少数值
    *错误（与以较低采样率在MCU上进行集成相比）。
    * DMP_FEATURE_6X_LP_QUAT：在DMP上生成一个陀螺/加速四元数
    * 200Hz。不能与DMP_FEATURE_LP_QUAT结合使用。
    * DMP_FEATURE_TAP：检测沿X，Y和Z轴的抽头。
    * DMP_FEATURE_ANDROID_ORIENT：Google的屏幕旋转算法。扳机
    *屏幕应在四个方向旋转的事件。
    * DMP_FEATURE_GYRO_CAL：在8秒钟后校准陀螺仪数据
    *不动。
    * DMP_FEATURE_SEND_RAW_ACCEL：将原始加速度计数据添加到FIFO。
    * DMP_FEATURE_SEND_RAW_GYRO：将原始陀螺仪数据添加到FIFO。
    * DMP_FEATURE_SEND_CAL_GYRO：将校准的陀螺仪数据添加到FIFO。不能
    *与DMP_FEATURE_SEND_RAW_GYRO结合使用。
    */
    dmp_load_motion_driver_firmware();
    dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
    dmp_register_tap_cb(tap_cb);
    dmp_register_android_orient_cb(android_orient_cb);
    /*
    *已知错误-
    * DMP启用后将以200Hz采样传感器数据并以该速率输出到FIFO
    *在dmp_set_fifo_rate API中指定。 然后，DMP将发送一次中断
    *样本已放入FIFO。 因此，如果dmp_set_fifo_rate为25Hz
    * MPU设备会有25Hz的中断。
    *
    *如果不启用DMP_FEATURE_TAP，则存在一个已知问题
    *则即使fifo速率中断也会达到200Hz
    *设置为其他比率。 为避免此问题，请包括DMP_FEATURE_TAP
    *
    * DMP传感器融合仅适用于+ -2000dps和加速+ -2G的陀螺仪
    */
    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
                       DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
                       DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    mpu_set_dmp_state(1);
    hal.dmp_on = 1;

    while(1)
    {

        unsigned long sensor_timestamp;
        int new_data = 0;
//		if (USART_GetITStatus(USART2, USART_IT_RXNE))
//		{
//			/*已通过USART接收到一个字节。 请参阅handle_input以获取以下内容的列表
//			*有效命令。
//			*/
//			USART_ClearITPendingBit(USART2, USART_IT_RXNE);
//			handle_input();
//		}
        get_tick_count(&timestamp);

#ifdef COMPASS_ENABLED
        /*我们没有为指南针使用数据就绪中断，因此我们将
			*使我们的指南针改为基于计时器。
			*/
			if ((timestamp > hal.next_compass_ms) && !hal.lp_accel_mode &&
			hal.new_gyro && (hal.sensors & COMPASS_ON))
			{
				hal.next_compass_ms = timestamp + COMPASS_READ_MS;
				new_compass = 1;
			}
#endif
        /*不需要每个陀螺仪样本都读取温度数据。
        *让我们使它们基于计时器，就像指南针一样。
        */
        if (timestamp > hal.next_temp_ms)
        {
            hal.next_temp_ms = timestamp + TEMP_READ_MS;
            new_temp = 1;
        }
        if (hal.motion_int_mode)
        {
            /*启用运动中断。 */
            mpu_lp_motion_interrupt(500, 1, 5);
            /*等待MPU中断。 */
            inv_accel_was_turned_off();
            inv_gyro_was_turned_off();
            inv_compass_was_turned_off();
            inv_quaternion_sensor_was_turned_off();
            /*等待MPU中断。 */
            while (!hal.new_gyro) {}
            /*恢复以前的传感器配置。 */
            mpu_lp_motion_interrupt(0, 0, 0);
            hal.motion_int_mode = 0;
        }

        if (!hal.sensors || !hal.new_gyro)
        {
            continue;
        }

        if (hal.new_gyro && hal.lp_accel_mode)
        {
            short accel_short[3];
            long accel[3];
            mpu_get_accel_reg(accel_short, &sensor_timestamp);
            accel[0] = (long)accel_short[0];
            accel[1] = (long)accel_short[1];
            accel[2] = (long)accel_short[2];
            inv_build_accel(accel, 0, sensor_timestamp);
            new_data = 1;
            hal.new_gyro = 0;
        } else if (hal.new_gyro && hal.dmp_on)
        {
            short gyro[3], accel_short[3], sensors;
            unsigned char more;
            long accel[3], quat[4], temperature;
            /*当DMP插入时，此函数从FIFO获取新数据
            *采用。 FIFO可以包含陀螺仪，加速度，
            *四元数和手势数据。传感器参数告诉
            *呼叫者实际在哪些数据字段中填充了新数据。
            *例如，如果传感器==（INV_XYZ_GYRO | INV_WXYZ_QUAT），则
            *FIFO未填充加速数据。
            *驱动程序解析手势数据以确定手势是否
            *事件已发生；在发生事件时，将通知该应用程序
            *通过回调（假设回调函数正确
            *已注册）。如果存在，则more参数为非零
            *FIFO中剩余的数据包。
            */
            dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
            if (!more)
                hal.new_gyro = 0;
            if (sensors & INV_XYZ_GYRO)
            {
                /*将新数据推送到MPL。 */
                inv_build_gyro(gyro, sensor_timestamp);
                new_data = 1;
                if (new_temp)
                {
                    new_temp = 0;
                    /*温度仅用于陀螺仪温度补偿。 */
                    mpu_get_temperature(&temperature, &sensor_timestamp);
                    inv_build_temp(temperature, sensor_timestamp);
                }
            }
            if (sensors & INV_XYZ_ACCEL)
            {
                accel[0] = (long)accel_short[0];
                accel[1] = (long)accel_short[1];
                accel[2] = (long)accel_short[2];
                inv_build_accel(accel, 0, sensor_timestamp);
                new_data = 1;
            }
            if (sensors & INV_WXYZ_QUAT)
            {
                inv_build_quat(quat, 0, sensor_timestamp);
                new_data = 1;
            }
        } else if (hal.new_gyro)
        {
            short gyro[3], accel_short[3];
            unsigned char sensors, more;
            long accel[3], temperature;
            /*此函数从FIFO获取新数据。 FIFO可以包含
            *陀螺仪，加速器，或两者都不选。传感器参数告诉
            *呼叫者实际在哪些数据字段中填充了新数据。
            *例如，如果传感器== INV_XYZ_GYRO，则FIFO不是
            *填充加速数据。 more参数为非零
            *FIFO中有剩余的数据包。 HAL可以使用此
            *信息以增加此功能的使用频率
            *称为。
            */
            hal.new_gyro = 0;
            mpu_read_fifo(gyro, accel_short, &sensor_timestamp,
                          &sensors, &more);
            if (more)
                hal.new_gyro = 1;
            if (sensors & INV_XYZ_GYRO)
            {
                /*将新数据推送到MPL。 */
                inv_build_gyro(gyro, sensor_timestamp);
                new_data = 1;
                if (new_temp)
                {
                    new_temp = 0;
                    /*温度仅用于陀螺仪温度补偿。 */
                    mpu_get_temperature(&temperature, &sensor_timestamp);
                    inv_build_temp(temperature, sensor_timestamp);
                }
            }
            if (sensors & INV_XYZ_ACCEL)
            {
                accel[0] = (long)accel_short[0];
                accel[1] = (long)accel_short[1];
                accel[2] = (long)accel_short[2];
                inv_build_accel(accel, 0, sensor_timestamp);
                new_data = 1;
            }
        }
#ifdef COMPASS_ENABLED
        if (new_compass)
			{
				short compass_short[3];
				long compass[3];
				new_compass = 0;
				/*对于在辅助I2C总线上具有AKM的任何MPU设备，原始
				*磁力计寄存器复制到特殊陀螺仪寄存器。
				*/
				if (!mpu_get_compass_reg(compass_short, &sensor_timestamp))
				{
					compass[0] = (long)compass_short[0];
					compass[1] = (long)compass_short[1];
					compass[2] = (long)compass_short[2];
					/*注意：如果使用第三方指南针校准库，
					*以uT传递罗盘数据*2 ^ 16并设置第二个
					*INV_CALIBRATED的参数| acc，其中acc是
					*精度从0到3。
					*/
					inv_build_compass(compass, 0, sensor_timestamp);
				}
				new_data = 1;
			}
#endif
        if (new_data)
        {
            inv_execute_on_data();
            /*此功能读取偏置补偿的传感器数据和传感器
            *MPL的融合输出。输出格式如所见
            *在eMPL_outputs.c中。此功能只需要在
            *房东要求的价格。
            */
            read_from_mpl();
        }
    }
}




void mdelay(unsigned long nTime)
{
	HAL_Delay(nTime);
}

int get_tick_count(unsigned long *count)
{
        count[0] = HAL_GetTick();
	return 0;
}
