/*
 * @Author: houzhinan 
 * @Date: 2022-01-04 15:18:01 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2022-01-04 20:20:37
 */

#ifndef LOGGER_H
#define LOGGER_H

#include "spdlog/spdlog.h"
#include "spdlog/sinks/daily_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"

/*
DEBUG_LOGGER: debug logger
LOGGER: console logger
MPC_LOGGER: mpc calculation details
*/

#define LOGGER_FILENAME "./logs/log.txt"
#define MPC_LOGGER_FILENAME "./logs/mpc_log.txt"

extern std::shared_ptr<spdlog::sinks::daily_file_sink_mt> DAILY_SINK;
extern std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> CONSOLE_SINK;
extern std::shared_ptr<spdlog::sinks::basic_file_sink_mt> MPC_SINK;

extern spdlog::sinks_init_list SINK_LIST;
extern spdlog::logger LOGGER;
extern spdlog::logger DEBUG_LOGGER;
extern spdlog::logger MPC_LOGGER;


void logger_config();

#endif