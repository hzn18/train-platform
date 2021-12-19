/*
 * @Author: houzhinan 
 * @Date: 2021-12-19 15:10:28 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2021-12-19 18:10:01
 */

#ifndef PREDICTOR_H
#define PREDICTOR_H

#include <vector>

std::vector<std::vector<double>> MBPredictor(double space);

std::vector<std::vector<double>> NPPredictor(double space, double speed);

std::vector<std::vector<double>> SHPredictor(std::vector<std::vector<double>> leader_mpc_result);

#endif