/*
 * @Author: houzhinan 
 * @Date: 2022-01-04 15:52:52 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2022-01-04 16:32:39
 */

#ifndef FILENAME_H
#define FILENAME_H

/********************  dynamic progarmming  ********************/
// #define DB_FILENAME "../../db/speed_limit.txt" // simluation filename
#define DB_FILENAME "./db/speed_limit_test.txt" //automation test
#define DP_SAFE_OUTPUT_FILENAME "./user/result/DP/dp_safe_result.txt"
#define DP_REF_OUTPUT_FILENAME "./user/result/DP/dp_ref_result.txt"

/********************  leader control  ********************/
#define LEADER_OUTPUT_FILENAME "./user/result/MPC/leader_result.txt"
#define DATA_DRIVEN_LEADER_OUTPUT_DIR "./user/result/DataDrivenMPC/"

/********************  convoy control  ********************/
#define FOLLOW_OUTPUT_DIR = "./user/result";

#endif