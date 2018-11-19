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

extern volatile short Yaw;
extern volatile short Pitch;

void listen2car();
void publish2car(const vision_mul::armor_pos& pos,short, short);

} // namespace serial_mul
} // namespace autocar

#endif