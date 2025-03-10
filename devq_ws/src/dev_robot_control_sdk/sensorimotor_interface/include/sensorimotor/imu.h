#pragma once

// stddef.h：标准C库头文件，提供了常用的类型定义和宏
// - 定义了 size_t 类型：用于表示内存大小和数组索引
// - 定义了 NULL 指针常量
// - 定义了 offsetof 宏：用于计算结构体成员的偏移量
#include <stddef.h>

// stdint.h：标准C库头文件，提供了固定大小的整数类型定义
// - 提供如 uint8_t（8位无符号整数）
// - 提供如 int16_t（16位有符号整数）
// - 提供如 uint32_t（32位无符号整数）
// - 提供如 int64_t（64位有符号整数）
// 这些类型在不同平台上都具有相同的大小，确保了代码的可移植性
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(1)

/**
 * IMU（惯性测量单元）数据结构
 * 用于存储IMU传感器的测量数据，包括加速度、角速度、姿态角和四元数
 */
typedef struct
{
    // 加速度数据，单位：米/平方秒（m/s²）
    // 分别表示在X、Y、Z三个轴向上的线性加速度
	float acc_x;  // X轴加速度
	float acc_y;  // Y轴加速度
	float acc_z;  // Z轴加速度

    // 角速度数据，单位：弧度/秒（rad/s）
    // 分别表示绕X、Y、Z三个轴的旋转速度
	float gyro_x;  // 绕X轴角速度
	float gyro_y;  // 绕Y轴角速度
	float gyro_z;  // 绕Z轴角速度

    // 欧拉角姿态数据，单位：弧度（rad）
    // 使用欧拉角表示IMU的当前姿态
	float pitch;   // 俯仰角：绕Y轴旋转的角度
	float roll;    // 横滚角：绕X轴旋转的角度
	float yaw;     // 偏航角：绕Z轴旋转的角度
	
    // 四元数姿态数据
    // 使用四元数表示IMU的当前姿态，相比欧拉角可以避免万向节死锁问题
	float quaternion_w;  // 四元数实部
	float quaternion_x;  // 四元数虚部i
	float quaternion_y;  // 四元数虚部j
	float quaternion_z;  // 四元数虚部k
} imu_data_t;

#pragma pack()

/**
 * IMU数据回调函数类型定义
 * 用于在新的IMU数据可用时进行异步通知
 * 
 * @param data   指向IMU数据结构的指针，包含最新的传感器数据
 * @param param  用户自定义参数指针，可用于传递额外的上下文信息
 */
typedef void (*imu_data_callback_t)(const imu_data_t *data, void *param);

/**
 * 初始化IMU设备
 * 打开并配置IMU设备，可选择是否进行姿态校准
 * 
 * @param device_path     IMU设备的路径（如：/dev/ttyUSB0）
 * @param calibrate       是否执行IMU姿态校准，默认为false
 * @param callback        IMU数据回调函数，用于接收实时数据，默认为NULL
 * @param callback_param  传递给回调函数的用户参数，默认为NULL
 * @return               成功返回0，失败返回其他值
 */
int initialize_imu(const char *device_path, bool calibrate=false, 
	imu_data_callback_t callback=NULL, void *callback_param=NULL);

/**
 * 关闭IMU设备
 * 释放所有资源并关闭IMU设备
 */
void deinitialize_imu();

/**
 * 获取当前IMU数据
 * 同步读取IMU的最新数据
 * 
 * @param data  指向IMU数据结构的指针，用于存储读取到的数据
 * @return     成功返回0，失败返回其他值
 */
int get_imu_data(imu_data_t *data);


#ifdef __cplusplus
}
#endif
