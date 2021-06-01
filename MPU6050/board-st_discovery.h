#ifndef __ST_DISCOVERY_BOARD_H
#define __ST_DISCOVERY_BOARD_H
//外部中断初始化

int get_tick_count(unsigned long *count);
//延时函数
void mdelay(unsigned long nTime);
void mpu_test(void);
void gyro_data_ready_cb(void);
#endif	/* ST_DISCOVERY_BOARD_H */