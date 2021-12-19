/*
 * @Author: houzhinan 
 * @Date: 2021-12-19 20:47:02 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2021-12-19 20:47:25
 */

#ifndef _READ_SPEED_LIMIT_H_
#define _READ_SPEED_LIMIT_H_

#include <string>
#include <vector>

std::vector<std::pair<double, double>> ReadSpeedLimit(std::string speed_limit_file);

#endif