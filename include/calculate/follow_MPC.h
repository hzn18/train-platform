/*
 * @Author: houzhinan 
 * @Date: 2021-12-19 15:04:29 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2021-12-19 19:17:03
 */

#ifndef FOLLOW_MPC_H
#define FOLLOW_MPC_H

#include <vector>
#include "spdlog/spdlog.h"


std::vector<std::vector<double>> FollowMPCCalculate(std::vector<std::vector<double>> predictor, double space, double speed, std::vector<std::pair<double, double>> speed_max_info_part, std::string mpc_filename, spdlog::logger logger);

#endif