/*******************************************************************************************************************
Copyright 2017 Dajiang Innovations Technology Co., Ltd (DJI)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
documentation files(the "Software"), to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense, and / or sell copies of the Software, and 
to permit persons to whom the Software is furnished to do so, subject to the following conditions : 

The above copyright notice and this permission notice shall be included in all copies or substantial portions of
the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
*******************************************************************************************************************/

/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2018, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#pragma once
#include <list>
#include <chrono>
#include "detect_factory/armor_info.h"
#include "slover/angle_slover.hpp"

namespace autocar
{
namespace slover
{

template<typename TargetType>
class Predictor
{
    const int history_size_;

    std::list<double> history_time;
    std::list<TargetType> history_Target;

    int miss_detection_cnt;

public:
    Predictor(int size_ = 5): history_size_(size_)
    {
    }
    void setLastResult(TargetType Target, double cost_time)
    {
        if (history_Target.size() < history_size_)
        {
            history_Target.push_back(Target);
            history_time.push_back(cost_time);
        }
        else {
            history_Target.push_back(Target);           
            history_time.push_back(cost_time);
            history_Target.pop_front();
            history_time.pop_front();
        }
    }
    void clear()
    {
        // history_Target.clear();
        // history_time.clear();
        if (history_Target.size() && history_time.size())
        {
            history_Target.pop_front();
            history_time.pop_front();
        }
    }
    virtual TargetType predict(); // predict new Target from history Target List
};

} // namespace slover
} // namespace autocar