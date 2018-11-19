/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2018, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */
#include "slover/armor_recorder.hpp"
#include "slover/angle_slover.hpp"
#include "utility/debug_utility.hpp"
namespace autocar
{
namespace slover
{

#define show_src_rect

const int bullet_speed = 18;     
const cv::Point ptoffset = cv::Point(22,0); // 子弹的偏移量 offset x → y ↓  

/**
 * @brief 定义一个距离函数，计算装甲片之间姿态的“距离”
 *
inline bool pos_distance(const vision_mul::armor_pos& pos1, const vision_mul::armor_pos& last_pos )
{
    return std::sqrt((pos1.angle_x - last_pos.angle_x) * (pos1.angle_x - last_pos.angle_x) + 
                     (pos1.angle_y - last_pos.angle_y) * (pos1.angle_y - last_pos.angle_y) +
                      std::abs(pos1.angle_z - last_pos.angle_z))/10;   
}

vision_mul::armor_pos Armor_recorder::SlectFinalArmor(std::vector<vision_mul::armor_info> &armors, AngleSolver& angle_slover, AngleSolverFactory& angle_slover_factory, cv::Mat & src) 
{
    std::vector<vision_mul::armor_pos> pos_vect;
#ifdef show_src_rect
    std::vector<vision_mul::armor_info> armor_vect;
#endif
    vision_mul::armor_pos armor_pos_;
    for (auto armor : armors)
    {
        double armor_ratio = std::max(armor.rect.size.width, armor.rect.size.height) / 
							 std::min(armor.rect.size.width, armor.rect.size.height);
        cv::RotatedRect rect = armor.rect;
        if (armor_ratio < 4)
        {
            if (angle_slover_factory.getAngle(rect, AngleSolverFactory::TARGET_SAMLL_ARMOR, armor_pos_.angle_x, armor_pos_.angle_y, bullet_speed, ptoffset ) == true)
            {
                this->miss_detection_cnt = 0;
                armor_pos_.Flag = armor.state; // [1 2 3 4]
                armor_pos_.angle_z = angle_slover._distance;
                pos_vect.push_back(armor_pos_);
                #ifdef show_src_rect
                armor_vect.push_back(armor);
                #endif
            }
            else{
                armor_pos_.Flag = 0;
                armor_pos_.angle_z = angle_slover._distance;
            }
        }
        else
        {
            if (angle_slover_factory.getAngle(rect, AngleSolverFactory::TARGET_ARMOR, armor_pos_.angle_x, armor_pos_.angle_y, bullet_speed, ptoffset ) == true)
            {
                this->miss_detection_cnt = 0;
                armor_pos_.Flag = armor.state; // [1 2 3 4]
                armor_pos_.angle_z = angle_slover._distance;
                pos_vect.push_back(armor_pos_);
                #ifdef show_src_rect
                armor_vect.push_back(armor);
                #endif
            }
            else{
                armor_pos_.Flag = 0;
                armor_pos_.angle_z = angle_slover._distance;
            }
        } // if infantry or hero
    } // for

    vision_mul::armor_pos last_pos;

    if (history_pos.size())
    {
        last_pos = history_pos.back();
    }
   
    if(pos_vect.size())
    {
        double dis_min = 100000000;
        int idx = 0;
        for (int i = 0; i != pos_vect.size(); ++i)
        {
            double dis = pos_distance(pos_vect[i],last_pos);
            if (dis < dis_min)
            {
                dis_min = dis;
                idx = i;
            }
        }
        #ifdef show_src_rect
             draw_rotated_rect(src, armor_vect[idx].rect,cv::Scalar(0,255,255),2);
        #endif
        return pos_vect[idx];
    }
    return vision_mul::armor_pos();
}*/

} // namespace slover
} // namepsace autocar
