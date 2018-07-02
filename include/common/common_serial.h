/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2018, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#ifndef COMMON_SERIAL_H
#define COMMON_SERIAL_H

#include <chrono>
#include <thread> 
#include "driver/LinuxSerial.hpp"
#include "detect_factory/armor_info.h"

namespace autocar
{
namespace serial_mul
{

short get_yaw();
short get_pitch();

void listen2car();
/**
 * @brief: 向串口发布armor_pos信息, 单摄像头版本
 */
void publish2car(const vision_mul::armor_pos& pos,short, short);

} // namespace serial_mul
} // namespace autocar

#endif