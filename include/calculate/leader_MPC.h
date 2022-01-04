/*
 * @Author: houzhinan 
 * @Date: 2021-12-19 13:11:44 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2022-01-04 15:32:33
 */

#ifndef LEADER_MPC_H
#define LEADER_MPC_H

#include <vector>

std::vector<std::vector<double>> LeaderMPCCalculate(double space, double speed, std::vector<std::pair<double, double>>& speed_max_info_part);

std::vector<std::vector<double>> DataDrivenLeaderMPCCalculate(double space, double speed, std::vector<std::vector<double>> sample_set);

#endif