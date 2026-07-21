#include <chrono>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "unitree/robot/channel/channel_factory.hpp"

#include "HardwareBridge.hpp"

namespace fs = std::filesystem;
using namespace unitree::robot;

int main(int argc, char **argv)
{
    const fs::path executable = fs::weakly_canonical(fs::absolute(argv[0]));
    const fs::path package_root = executable.parent_path().parent_path();
    const fs::path config_file = package_root / "params" / "pact_config.yaml";
    if (!fs::is_regular_file(config_file))
    {
        std::cerr << "PACT config file not found: " << config_file << std::endl;
        return 1;
    }

    const bool real_robot = argc >= 2;
    if (real_robot)
        ChannelFactory::Instance()->Init(0, argv[1]);
    else
        ChannelFactory::Instance()->Init(1, "lo");

    const auto now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(now);
    std::stringstream timestamp;
    timestamp << std::put_time(std::localtime(&time), "%Y%m%d-%H%M%S");
    const fs::path log_folder = package_root / "build" / "logs" / timestamp.str();
    fs::create_directories(log_folder);

    try
    {
        HardwareBridge hardware_bridge(config_file.string(), log_folder / "log.csv", real_robot);
        hardware_bridge.run();
    }
    catch (const std::exception &error)
    {
        std::cerr << "FATAL: " << error.what() << std::endl;
        return 1;
    }
    return 0;
}
