#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "unitree/idl/go2/LowState_.hpp"
#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"

using unitree::robot::ChannelFactory;
using unitree::robot::ChannelSubscriber;

int main(int argc, char **argv)
{
    if (argc < 2 || argc > 3)
    {
        std::cerr << "Usage: " << argv[0] << " <network-interface> [timeout-seconds]\n";
        return 2;
    }

    const std::string interface = argv[1];
    const int timeout_seconds = argc == 3 ? std::max(1, std::atoi(argv[2])) : 5;
    std::atomic<unsigned int> message_count{0};
    std::atomic<float> first_joint_position{0.0f};

    ChannelFactory::Instance()->Init(0, interface);
    auto subscriber = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::LowState_>>("rt/lowstate");
    subscriber->InitChannel(
        [&](const void *raw_message)
        {
            const auto *state = static_cast<const unitree_go::msg::dds_::LowState_ *>(raw_message);
            first_joint_position.store(state->motor_state()[0].q(), std::memory_order_relaxed);
            message_count.fetch_add(1, std::memory_order_relaxed);
        },
        1);

    std::cout << "Listening for rt/lowstate on " << interface << " for "
              << timeout_seconds << " seconds..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(timeout_seconds));

    const unsigned int received = message_count.load(std::memory_order_relaxed);
    const float joint_position = first_joint_position.load(std::memory_order_relaxed);

    // Stop DDS callbacks and release the SDK resources before the callback's
    // stack-owned state is destroyed.
    subscriber->CloseChannel();
    subscriber.reset();
    ChannelFactory::Instance()->Release();

    if (received == 0)
    {
        std::cerr << "FAIL: no Go2 low-state messages received. Check the interface, cable, "
                     "IP configuration, firewall, and SDK2 installation.\n";
        return 1;
    }

    std::cout << "PASS: received " << received << " low-state messages; motor 0 q = "
              << joint_position << std::endl;
    return 0;
}
