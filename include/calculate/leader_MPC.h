/*
 * @Author: houzhinan 
 * @Date: 2021-12-19 13:11:44 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2021-12-19 18:23:23
 */

#ifndef LEADER_MPC_H
#define LEADER_MPC_H

#include <vector>
#include "spdlog/spdlog.h"


std::vector<std::vector<double>> LeaderMPCCalculate(double space, double speed, std::vector<std::pair<double, double>>& speed_max_info_part, std::string mpc_filename ,spdlog::logger& logger);

#endif