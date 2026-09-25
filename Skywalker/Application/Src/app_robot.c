/**
 * @file app_robot.c
 * @brief 机器人具体业务层实现
 * @attention 
 * 这个文件包含了机器人各个功能模块的具体业务逻辑实现，所有的具体业务逻辑都在这里进行处理。
 * 不要修改其他文件的内容，如果需要框架不支持的功能，请在这里实现。
 * 如果后续需要复用，可以在提出Issue后，进行代码的重构和优化。
 */

#include "app_task.h"

uint32_t ManagerTaskInit(void)
{
  return 1U;
}

void ManagerTaskLoop(void)
{
}

uint32_t GimbalTaskInit(void)
{
  return 1U;
}

void GimbalTaskLoop(void)
{
}

uint32_t ChassisTaskInit(void)
{
  return 1U;
}

void ChassisTaskLoop(void)
{
}

uint32_t ShootTaskInit(void)
{
  return 1U;
}

void ShootTaskLoop(void)
{
}
