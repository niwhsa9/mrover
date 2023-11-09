#pragma once

#include <bitset>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <format>
#include <iostream>
#include <span>
#include <thread>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <nodelet/nodelet.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <boost/asio/io_service.hpp>
#include <boost/asio/posix/basic_stream_descriptor.hpp>

#include <bimap.hpp>
#include <mrover/CAN.h>

#include "can_net_link.hpp"

// TODO(owen): support multiple buses

namespace mrover {

    // [0-28]: CAN identifier (11/29bit)
    // [29]: Error frame flag (0 = data frame, 1 = error frame)
    // [30]: Remote transmission request flag (1 = rtr frame)
    // [31]: Frame format flag (0 = standard 11bit, 1 = extended 29bit)
    // In the future, if we want to send different types of messages,
    // we should have logic for switching bits such as errorFrameFlag.
    struct RawCanFdId {
        std::uint32_t identifier : 29 {};
        bool isErrorFrame : 1 {};
        bool isRemoteTransmissionRequest : 1 {};
        bool isExtendedFrame : 1 {};
    };
    static_assert(sizeof(RawCanFdId) == sizeof(canid_t));

    struct CanFdMessageId {
        std::uint8_t destination{};
        std::uint8_t source : 7 {};
        bool replyRequired : 1 {};
    };
    static_assert(sizeof(CanFdMessageId) == 2);

    struct CanFdAddress {
        std::uint8_t bus{};
        std::uint8_t id{};

        // "Spaceship" operator
        // See: https://devblogs.microsoft.com/cppblog/simplify-your-code-with-rocket-science-c20s-spaceship-operator/
        bool operator<=>(CanFdAddress const& other) const = default;
    };

    struct CanFdPubSub {
        ros::Publisher publisher;
        ros::Subscriber subscriber;
    };

    class CanNodelet : public nodelet::Nodelet {
    public:
        CanNodelet() = default;

        ~CanNodelet() override;

    private:
        ros::NodeHandle mNh, mPnh;

        std::string mInterface;
        bool mIsExtendedFrame{};
        std::uint32_t mBitrate{};
        std::uint32_t mBitratePrescaler{};

        canfd_frame mReadFrame{};
        CanNetLink mCanNetLink;
        std::optional<boost::asio::posix::basic_stream_descriptor<>> mStream;
        std::jthread mIoThread;
        boost::asio::io_service mIoService;

        bimap<std::string, CanFdAddress,
              std::hash<std::string>, decltype([](CanFdAddress const& address) {
                  return std::hash<std::uint8_t>{}(address.bus) ^ std::hash<std::uint8_t>{}(address.id);
              })>
                mDevices;
        std::unordered_map<std::string, CanFdPubSub> mDevicesPubSub;

        int setupSocket();

        void onInit() override;

        void readFrameAsync();

        void frameReadCallback();

        void frameSendRequestCallback(CAN::ConstPtr const& msg);
    };

} // namespace mrover
