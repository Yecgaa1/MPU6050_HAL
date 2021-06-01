# MPU6050的stm32的HAL库的移植教程：



### 第一步：

在**选择库**文件夹里面选择合适的结算静态库文件，解压一层（到后缀为.a的文件），来替换掉**MPU6050驱动\MPU6050\libmpllib**下的文件。

如我使用的CLion+GCC编译环境中，我选择是arm\gcc4.9.3下的文件

再根据我使用F1对应M3，F4对应M4，F7选择M4似乎也可以了

### 第二步：

将**MPU6050**文件夹添加进工程，并且开始修改cmake文件把对应的段落放在对应的相同位置，比如include_directories和别的include_directories放在一起

```cmake
add_definitions(-DMPL_LOG_NDEBUG=1 -DMPU6050 -DEMPL -DUSE_DMP -DEMPL_TARGET_STM32F4)#定义全局宏变量，按照下面的说明修改

include_directories(Core/MPU6050 Core/MPU6050/driver/eMPL Core/MPU6050/driver/include Core/MPU6050/driver/stm32L Core/MPU6050/mllite Core/MPU6050/eMPL-hal Core/MPU6050/mpl)#导入相对应的目录否则会include错误
link_directories(Core/MPU6050/libmpllib)#链接静态库文件.a，放在file,include_directories下

TARGET_LINK_LIBRARIES(${PROJECT_NAME}.elf libmplmpu)#放在add_executable后
```

在以下宏定义中选择合适的全局宏定义：

MPL_LOG_NDEBUG=1

MPU9150 or MPU6050 or MPU6500 or MPU9250

EMPL

USE_DMP

EMPL_TARGET_MSP430 or its equivalent 



比如我的选择是：

MPL_LOG_NDEBUG=1 （表示打印报告）

MPU6050                         (表示选择的是MPU6050)

EMPL                              

USE_DMP						（表示使用DMP）

EMPL_TARGET_STM32F4  （表示使用的是STMF系列，F1或者F7都是用这个全局宏）

然后定义在工程中定义这几个宏就行了。

（-D表示定义量，后面不需要加空格）

### 第四步(已经预先做了)：

在工程中编写底层I2C接口函数：
建议在MPU6050/driver/eMPL/inv_mpu.c
~~~ c

int Sensors_I2C_WriteRegister(unsigned char slave_addr,
                              unsigned char reg_addr,
                              unsigned short len,
                              const unsigned char *data_ptr)
{
    if(HAL_I2C_Mem_Write(&hi2c1 , slave_addr,reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)data_ptr,len,1000)!= HAL_OK)
    {
        /* Writing process Error */
        return HAL_ERROR;
    }

    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
    {
    }

    /* Check if the EEPROM is ready for a new operation */
    while (HAL_I2C_IsDeviceReady(&hi2c1, slave_addr, 100, 100) == HAL_TIMEOUT);

    /* Wait for the end of the transfer */
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
    {
    }
    return HAL_OK;
}


int Sensors_I2C_ReadRegister(unsigned char slave_addr,
                             unsigned char reg_addr,
                             unsigned short len,
                             unsigned char *data_ptr)
{
    if (HAL_I2C_Mem_Read(&hi2c1 , slave_addr,reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)data_ptr,len,1000) != HAL_OK)
    {
        /* Reading process Error */
        return HAL_ERROR;
    }

    /* Wait for the end of the transfer */
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
    {
    }
    return HAL_OK;
}
~~~

### 第五步:使用
配置正常的I2C口(cubemx不需要改任何参数)
和一个中断引脚用于获取dmp就绪的信号,该脚命名为MPU6050INT_Pin

以及中断函数（我这里是使用了MPU6050的中断脚功能，所以要加上这个）

~~~c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin==MPU6050INT_Pin)
    {
        gyro_data_ready_cb();
    }
}
~~~



在main函数里面添加

~~~c
 mpu_test();
~~~



关键的数据读取在：read_from_mpl函数中，这个函数里面被注释的代码也是可以根据自己的使用来进行调用。

## 贡献者

ljl

xutongxin

