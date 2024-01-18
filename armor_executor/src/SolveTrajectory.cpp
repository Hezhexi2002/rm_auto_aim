/*
@brief: 弹道解算 适配陈君的rm_vision
@author: CodeAlan  华南师大Vanguard战队
*/
// 近点只考虑水平方向的空气阻力



//TODO 完整弹道模型
//TODO 适配英雄机器人弹道解算


// STD
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>
#include <iostream>


#include "armor_executor/SolveTrajectory.hpp"

namespace rm_auto_aim
{
SolveTrajectory::SolveTrajectory(const float &k, const int &bias_time, const float &s_bias, const float &z_bias)
: k(k), bias_time(bias_time), s_bias(s_bias), z_bias(z_bias)
{}


void SolveTrajectory::init(const auto_aim_interfaces::msg::Velocity::SharedPtr velocity_msg)
{
  current_v = velocity_msg->velocity;
}

/*
@brief 单方向空气阻力弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rachouxiang
@return z:m
*/
float SolveTrajectory::monoDirectionalAirResistanceModel(float s, float v, float angle)
{
    float z, t;
    //t为给定v与angle时的飞行时间
    t = (float)((exp(k * s) - 1) / (k * v * cos(angle)));
    //z为给定v与angle时的高度
    z = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
    return z;
}


/*
@brief 完整弹道模型
@param s:m 距离
@param v:m/s 速度
@param angle:rad 角度
@return z:m
*/
//TODO 完整弹道模型
float SolveTrajectory::completeAirResistanceModel(float s, float v, float angle)
{
    // TODO: Implement complete air resistance model
    return 0.0f;
}



/*
@brief pitch轴解算
@param s:m 距离
@param z:m 高度
@param v:m/s
@return angle_pitch:rad
*/
float SolveTrajectory::pitchTrajectoryCompensation(float s, float z, float v) {
    float z_temp = z;
    float angle_pitch = 0.0f;
    std::cout << "current_velocity_:" << v << std::endl;
    for (int i = 0; i < 20; i++) {
        angle_pitch = std::atan2(z_temp, s);
        float z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
        float dz = 0.3f * (z - z_actual);
        z_temp += dz;

        if (std::fabs(dz) < 0.00001f) {
            break;
        }
    }

    return angle_pitch;
}

bool SolveTrajectory::shouldFire(float tmp_yaw, float v_yaw, float timeDelay) {
    return fabs((tmp_yaw + v_yaw * timeDelay) - 2 * PI) < 0.001;
}

void SolveTrajectory::calculateArmorPosition(const auto_aim_interfaces::msg::Target::SharedPtr& msg, bool use_1, bool use_average_radius) {
    std::vector<float> tmp_yaws;
    min_yaw_in_cycle = std::numeric_limits<float>::max();
    max_yaw_in_cycle = std::numeric_limits<float>::min();
    for (int i = 0; i < msg->armors_num; i++) {
        float tmp_yaw = tar_yaw + i * 2.0 * PI / msg->armors_num;
        tmp_yaws.push_back(tmp_yaw);
        min_yaw_in_cycle = std::min(min_yaw_in_cycle, tmp_yaw);
        max_yaw_in_cycle = std::max(max_yaw_in_cycle, tmp_yaw);
        float r;
        if (use_average_radius) {
            // 使用两个半径的平均值
            r = (msg->radius_1 + msg->radius_2) / 2;
        } else {
            // 使用r1或r2
            r = use_1 ? msg->radius_1 : msg->radius_2;
        }
        tar_position[i].x = msg->position.x - r * cos(tmp_yaw);
        tar_position[i].y = msg->position.y - r * sin(tmp_yaw);
        tar_position[i].z = use_1 ? msg->position.z : msg->position.z + msg->armors_num;
        tar_position[i].yaw = tmp_yaw;
        use_1 = !use_1;
    }
}

std::pair<float, float> SolveTrajectory::calculatePitchAndYaw(int idx, const auto_aim_interfaces::msg::Target::SharedPtr& msg, float timeDelay, float s_bias, float z_bias, float current_v, bool use_target_center_for_yaw) {
    float aim_x = tar_position[idx].x  + msg->velocity.x * timeDelay;
    float aim_y = tar_position[idx].y  + msg->velocity.y * timeDelay;
    float aim_z = tar_position[idx].z  + msg->velocity.z * timeDelay;

    float yaw_x = use_target_center_for_yaw ? msg->position.x : aim_x;
    float yaw_y = use_target_center_for_yaw ? msg->position.y : aim_y;

    float pitch = -pitchTrajectoryCompensation(sqrt((aim_x) * (aim_x) + (aim_y) * (aim_y)) - s_bias,
            aim_z + z_bias, current_v);
    float yaw = (float)(atan2(yaw_y, yaw_x));

    return std::make_pair(pitch, yaw);
}

int SolveTrajectory::selectArmor(const auto_aim_interfaces::msg::Target::SharedPtr& msg, bool select_by_min_yaw) {
    int selected_armor_idx = -1;
    if (select_by_min_yaw) {
        // 选择枪管到目标装甲板yaw最小的那dz个装甲板
        float min_yaw_diff = fabs(msg->yaw - tar_position[0].yaw);
        for (int i = 1; i < msg->armors_num; i++) {
            float temp_yaw_diff = fabs(msg->yaw - tar_position[i].yaw);
            if (temp_yaw_diff < min_yaw_diff) {
                min_yaw_diff = temp_yaw_diff;
                selected_armor_idx = i;
            }
        }
    } else {
        // 选择离你的机器人最近的装甲板
        float min_distance = std::numeric_limits<float>::max();
        for (int i = 0; i < msg->armors_num; i++) {
            float distance = sqrt(tar_position[i].x * tar_position[i].x + tar_position[i].y * tar_position[i].y + tar_position[i].z * tar_position[i].z);
            if (distance < min_distance) {
                min_distance = distance;
                selected_armor_idx = i;
            }
        }
    }

    return selected_armor_idx;
}

void SolveTrajectory::fireLogicIsTop(float& pitch, float& yaw, float& aim_x, float& aim_y, float& aim_z, const auto_aim_interfaces::msg::Target::SharedPtr& msg) {
    tar_yaw = msg->yaw;
    // 线性预测
    float timeDelay = bias_time/1000.0 + fly_time;


    //计算四块装甲板的位置
    //装甲板id顺序，以四块装甲板为例，逆时针编号
    //      2
    //   3     1
    //      0
    int idx = 0;
    bool is_fire = false;
    if (msg->armors_num  == ARMOR_NUM_BALANCE) {
        calculateArmorPosition(msg , true, false);
        for (size_t i = 0; i < tmp_yaws.size(); i++) {
            float tmp_yaw = tmp_yaws[i];
            if (shouldFire(tmp_yaw, msg->v_yaw, timeDelay)) {
                is_fire = true;
                idx = i;
                if (fireCallback) {
                    fireCallback(is_fire);
                }
                break;
            }
        }   
    } else if (msg->armors_num == ARMOR_NUM_OUTPOST) {
        calculateArmorPosition(msg, false, true);
        for (size_t i = 0; i < tmp_yaws.size(); i++) {
            float tmp_yaw = tmp_yaws[i];
            if (shouldFire(tmp_yaw, msg->v_yaw, timeDelay)) {
                is_fire = true;
                idx = i;
                if (fireCallback) {
                    fireCallback(is_fire);
                }
                break;
            }
        }   
    } else {
        calculateArmorPosition(msg, false, false);
        for (size_t i = 0; i < tmp_yaws.size(); i++) {
            float tmp_yaw = tmp_yaws[i];
            if (shouldFire(tmp_yaw, msg->v_yaw, timeDelay)) {
                is_fire = true;
                idx = i;
                if (fireCallback) {
                    fireCallback(is_fire);
                }
                break;
            }
        }   
    }

    auto pitch_and_yaw = calculatePitchAndYaw(idx, msg, timeDelay, s_bias, z_bias, current_v, false);
    pitch = pitch_and_yaw.first;
    yaw = pitch_and_yaw.second;


}

void SolveTrajectory::fireLogicDefault(float& pitch, float& yaw, float& aim_x, float& aim_y, float& aim_z, const auto_aim_interfaces::msg::Target::SharedPtr& msg) {

    // 线性预测
    float timeDelay = bias_time/1000.0 + fly_time;
    tar_yaw += msg->v_yaw * timeDelay;


    //计算四块装甲板的位置
    //装甲板id顺序，以四块装甲板为例，逆时针编号
    //      2
    //   3     1
    //      0
    int idx = 0;
    bool is_fire = false;
    if (msg->armors_num  == ARMOR_NUM_BALANCE) {
        calculateArmorPosition(msg, true, false);
        for (size_t i = 0; i < tmp_yaws.size(); i++) {
            idx = selectArmor(msg, true);
            is_fire = tmp_yaws[idx] >= min_yaw_in_cycle && tmp_yaws[idx] <= max_yaw_in_cycle;
            if (fireCallback) {
                fireCallback(is_fire);
            } 
        }   
    } else if (msg->armors_num == ARMOR_NUM_OUTPOST) {
        calculateArmorPosition(msg, false, true);
        for (size_t i = 0; i < tmp_yaws.size(); i++) {
            idx = selectArmor(msg, true);
            is_fire = tmp_yaws[idx] >= min_yaw_in_cycle && tmp_yaws[idx] <= max_yaw_in_cycle;
            if (fireCallback) {
                fireCallback(is_fire);
            } 
        }   
    } else {
        calculateArmorPosition(msg, false, false);
        for (size_t i = 0; i < tmp_yaws.size(); i++) {
            idx = selectArmor(msg, true);
            is_fire = tmp_yaws[idx] >= min_yaw_in_cycle && tmp_yaws[idx] <= max_yaw_in_cycle;
            if (fireCallback) {
                fireCallback(is_fire);
            } 
        }   
    }

    auto pitch_and_yaw = calculatePitchAndYaw(idx, msg, timeDelay, s_bias, z_bias, current_v, false);
    pitch = pitch_and_yaw.first;
    yaw = pitch_and_yaw.second;

}

/*
@brief 根据最优决策得出被击打装甲板 自动解算弹道
@param pitch:rad  传出pitch
@param yaw:rad    传出yaw
@param aim_x:传出aim_x  打击目标的x
@param aim_y:传出aim_y  打击目标的y
@param aim_z:传出aim_z  打击目标的z
*/
void SolveTrajectory::autoSolveTrajectory(float& pitch, float& yaw, float& aim_x, float& aim_y, float& aim_z, const auto_aim_interfaces::msg::Target::SharedPtr msg)
{

    // if(msg->v_yaw > 6.0f){
    //     fireLogicIsTop(pitch, yaw, aim_x, aim_y, aim_z, msg);
    // }
    // else{
    //     fireLogicDefault(pitch, yaw, aim_x, aim_y, aim_z, msg);
    // }
    fireLogicIsTop(pitch, yaw, aim_x, aim_y, aim_z, msg);

}

// 从坐标轴正向看向原点，逆时针方向为正

}
