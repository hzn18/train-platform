/*
 * @Author: houzhinan 
 * @Date: 2021-12-19 12:49:32 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2021-12-19 14:32:38
 */

#include <vector>

// Giving the controlled driving or braking force, this function will transform 
// state of train. 
// Return: train state [s, v]
// TODO: 单质点模型改为多质点模型，接口需要换一下
std::vector<double> DynamicModel(double function, double space, double speed);
