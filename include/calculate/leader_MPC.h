/*
 * @Author: houzhinan 
 * @Date: 2021-12-19 13:11:44 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2021-12-19 14:43:50
 */

#include <vector>
#include "spdlog/spdlog.h"


std::vector<std::vector<double>> LeaderMPCCaculate(double space, double speed, std::vector<std::pair<double, double>>& speed_max_info_part, std::string mpc_filename ,spdlog::logger& logger);