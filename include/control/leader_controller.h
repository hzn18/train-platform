/*
 * @Author: houzhinan 
 * @Date: 2021-12-19 12:59:52 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2021-12-19 14:36:45
 */

#ifndef LEADER_CONTROLLER_H
#define LEADER_CONTROLLER_H

#include <vector>
#include "spdlog/spdlog.h"

// Giving environment information and train state, we calculate
// the function.
std::vector<std::vector<double>> LeaderController(double space, double speed, std::vector<std::pair<double, double>>& speed_max_info, int speed_max_info_index, std::string mpc_filename ,spdlog::logger& logger);

#endif