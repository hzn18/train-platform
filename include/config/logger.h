/*
 * @Author: houzhinan 
 * @Date: 2022-01-04 15:18:01 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2022-01-04 16:39:56
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

#define LOGGER_FILENAME "../../logs/log.txt"
#define MPC_LOGGER_FILENAME "../../logs/mpc_log.txt"

auto DAILY_SINK = std::make_shared<spdlog::sinks::daily_file_sink_mt>(LOGGER_FILENAME, 23, 59);
auto CONSOLE_SINK = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

spdlog::sinks_init_list SINK_LIST = { DAILY_SINK, CONSOLE_SINK};
spdlog::logger LOGGER("console", SINK_LIST.begin(), SINK_LIST.end());

auto DEBUG_LOGGER = spdlog::logger("leader_debug", DAILY_SINK);

auto MPC_SINK = std::make_shared<spdlog::sinks::basic_file_sink_mt>(MPC_LOGGER_FILENAME);
auto MPC_LOGGER = spdlog::logger("mpc_log", MPC_SINK);


void logger_config(){
    CONSOLE_SINK->set_level(spdlog::level::info);
    LOGGER.set_level(spdlog::level::info);
    DEBUG_LOGGER.set_level(spdlog::level::debug);
    MPC_LOGGER.set_level(spdlog::level::debug);
    MPC_LOGGER.flush_on(spdlog::level::info);
}

#endif