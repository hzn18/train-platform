/*
 * @Author: houzhinan 
 * @Date: 2021-12-19 15:16:16 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2022-01-04 15:33:31
 */

#ifndef FOLLOW_CONTROLLER_H
#define FOLLOW_CONTROLLER_H

#include <vector>

// Giving environment information and train state, we calculate
// the function.
std::vector<std::vector<double>> FollowController(double leader_space, double space, double speed, std::vector<std::pair<double, double>> speed_max_info, int speed_max_info_index);

std::vector<std::vector<double>> FollowController(double leader_space, double leader_speed, double space, double speed, std::vector<std::pair<double, double>> speed_max_info, int speed_max_info_index);

std::vector<std::vector<double>> FollowController(std::vector<std::vector<double>> leader_mpc_result, double space, double speed, std::vector<std::pair<double, double>> speed_max_info, int speed_max_info_index);
    

#endif

