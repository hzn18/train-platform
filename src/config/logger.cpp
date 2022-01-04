/*
 * @Author: houzhinan 
 * @Date: 2022-01-04 19:57:39 
 * @Last Modified by: houzhinan
 * @Last Modified time: 2022-01-04 20:20:16
 */

#include "logger.h"

using namespace std;

shared_ptr<spdlog::sinks::daily_file_sink_mt> DAILY_SINK = make_shared<spdlog::sinks::daily_file_sink_mt>(LOGGER_FILENAME, 23, 59);
shared_ptr<spdlog::sinks::stdout_color_sink_mt> CONSOLE_SINK = make_shared<spdlog::sinks::stdout_color_sink_mt>();
shared_ptr<spdlog::sinks::basic_file_sink_mt> MPC_SINK = make_shared<spdlog::sinks::basic_file_sink_mt>(MPC_LOGGER_FILENAME);

spdlog::sinks_init_list SINK_LIST = { DAILY_SINK, CONSOLE_SINK};
spdlog::logger LOGGER("console", SINK_LIST.begin(), SINK_LIST.end());
spdlog::logger DEBUG_LOGGER = spdlog::logger("leader_debug", DAILY_SINK);
spdlog::logger MPC_LOGGER = spdlog::logger("mpc_log", MPC_SINK);


void logger_config(){
    CONSOLE_SINK->set_level(spdlog::level::info);
    LOGGER.set_level(spdlog::level::info);
    DEBUG_LOGGER.set_level(spdlog::level::debug);
    MPC_LOGGER.set_level(spdlog::level::debug);
    MPC_LOGGER.flush_on(spdlog::level::info);
}


