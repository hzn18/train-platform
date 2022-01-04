/*
 * @Author: houzhinan 
 * @Date: 2021-12-19 12:59:52 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2022-01-04 14:46:39
 */

#ifndef LEADER_CONTROLLER_H
#define LEADER_CONTROLLER_H

#include <vector>
#include "spdlog/spdlog.h"

// Giving environment information and train state, we calculate
// the function.
std::vector<std::vector<double>> LeaderController(double space, double speed, std::vector<std::pair<double, double>>& speed_max_info, int speed_max_info_index, std::string mpc_filename ,spdlog::logger& logger);

// sample_safe_set:
// 1st-dim: index
// 2nd-dim: space, speed, terminal_cost
std::vector<std::vector<double>> DataDrivenLeaderController(double space, double speed, std::vector<std::vector<double>>& sample_safe_set, int sample_safe_set_start_index, std::string mpc_filename ,spdlog::logger& logger);

#endif