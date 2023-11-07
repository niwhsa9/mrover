#include "can_node.hpp"

#include <linux/can.h>

#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>
#include <boost/system/error_code.hpp>

#include <nodelet/loader.h>
#include <ros/init.h>
#include <ros/names.h>
#include <ros/this_node.h>

namespace mrover {

    static int checkSyscallResult(int result) {
        if (result < 0) throw std::system_error{errno, std::generic_category()};

        return result;
    }

    static void checkErrorCode(boost::system::error_code const& ec) {
        if (ec.value() == boost::system::errc::success) return;

        throw std::runtime_error(std::format("Boost failure: {} {}", ec.value(), ec.message()));
    }

    static std::uint8_t nearestFittingFdcanFrameSize(std::size_t size) {
        if (size <= 8) return size;
        if (size <= 12) return 12;
        if (size <= 16) return 16;
        if (size <= 20) return 20;
        if (size <= 24) return 24;
        if (size <= 32) return 32;
        if (size <= 48) return 48;
        if (size <= 64) return 64;
        throw std::runtime_error("Too large!");
    }

    void CanNodelet::onInit() {
        try {
            NODELET_INFO("CAN Node starting...");

            mNh = getMTNodeHandle();
            mPnh = getMTPrivateNodeHandle();

            mInterface = mPnh.param<std::string>("interface", "can0");
            mIsExtendedFrame = mPnh.param<bool>("is_extended_frame", true);
            mBitrate = static_cast<std::uint32_t>(mPnh.param<int>("bitrate", 500000));
            mBitratePrescaler = static_cast<std::uint32_t>(mPnh.param<int>("bitrate_prescaler", 2));

            mCanSubscriber = mNh.subscribe<CAN>("can_requests", 16, &CanNodelet::frameSendRequestCallback, this);
            mCanPublisher = mNh.advertise<CAN>("can_data", 16);

            mCanNetLink = {mInterface, mBitrate, mBitratePrescaler};

            int socketFileDescriptor = setupSocket();
            mStream.emplace(mIoService);
            mStream->assign(socketFileDescriptor);

            readFrameAsync();

            // Since "onInit" needs to return, kick off a self-joining thread to run the IO concurrently
            mIoThread = std::jthread{[this] { mIoService.run(); }};

            NODELET_INFO("CAN Node started");

        } catch (std::exception const& exception) {
            NODELET_ERROR_STREAM("Exception in initialization: " << exception.what());
            ros::shutdown();
        }
    }

    int CanNodelet::setupSocket() {
        int socketFd = checkSyscallResult(socket(PF_CAN, SOCK_RAW, CAN_RAW));
        NODELET_INFO_STREAM("Opened CAN socket with file descriptor: " << socketFd);

        ifreq ifr{};
        std::strcpy(ifr.ifr_name, mInterface.c_str());
        ioctl(socketFd, SIOCGIFINDEX, &ifr);

        sockaddr_can addr{
                .can_family = AF_CAN,
                .can_ifindex = ifr.ifr_ifindex,
        };
        checkSyscallResult(bind(socketFd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)));
        NODELET_INFO("Bound CAN socket");

        int enableCanFd = 1;
        checkSyscallResult(setsockopt(socketFd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enableCanFd, sizeof(enableCanFd)));

        return socketFd;
    }

    void CanNodelet::readFrameAsync() { // NOLINT(*-no-recursion)
        // You would think we would have to read the header first to find the data length (which is not always 64 bytes) and THEN read the data
        // However socketcan is nice and just requires we read the max length
        // It then puts the actual length in the header
        boost::asio::async_read(
                mStream.value(),
                boost::asio::buffer(&mReadFrame, sizeof(mReadFrame)),
                // Supply lambda that is called on completion
                [this](boost::system::error_code const& ec, std::size_t bytes) { // NOLINT(*-no-recursion)
                    checkErrorCode(ec);
                    assert(bytes == sizeof(mReadFrame));

                    frameReadCallback();

                    // Ready for the next frame, start listening again
                    // Note this is recursive, but it is safe because it is async
                    // i.e. the stack is not growing
                    readFrameAsync();
                });
    }

    void CanNodelet::frameReadCallback() { // NOLINT(*-no-recursion)
        CAN msg;
        msg.bus = 0; // TODO(owen) support multiple buses
        msg.message_id = std::bit_cast<RawCanfdId>(mReadFrame.can_id).identifier;
        msg.data.assign(mReadFrame.data, mReadFrame.data + mReadFrame.len);

        ROS_DEBUG_STREAM("Received CAN message:\n"
                         << msg);

        mCanPublisher.publish(msg);
    }

    void CanNodelet::frameSendRequestCallback(CAN::ConstPtr const& msg) {
        ROS_DEBUG_STREAM("Received request to send CAN message:\n"
                         << *msg);

        // Check that the identifier is not too large
        assert(std::bit_width(msg->message_id) <= mIsExtendedFrame ? CAN_EFF_ID_BITS : CAN_SFF_ID_BITS);

        // Craft the SocketCAN frame from the ROS message
        canfd_frame frame{
                .can_id = std::bit_cast<canid_t>(RawCanfdId{
                        .identifier = msg->message_id,
                        .isExtendedFrame = mIsExtendedFrame,
                }),
                .len = nearestFittingFdcanFrameSize(msg->data.size()),
        };
        std::memcpy(frame.data, msg->data.data(), msg->data.size());

        std::size_t written = boost::asio::write(mStream.value(), boost::asio::buffer(std::addressof(frame), sizeof(frame)));
        if (written != sizeof(frame)) throw std::runtime_error("Failed to write entire CAN frame");

        ROS_DEBUG_STREAM("Sent CAN message");
    }

    CanNodelet::~CanNodelet() {
        mIoService.stop(); // This causes the io thread to finish
    }

} // namespace mrover

int main(int argc, char** argv) {
    ros::init(argc, argv, "can_node");

    // Start the ZED Nodelet
    nodelet::Loader nodelet;
    nodelet.load(ros::this_node::getName(), "mrover/CanNodelet", ros::names::getRemappings(), {});

    ros::spin();

    return EXIT_SUCCESS;
}

#ifdef MROVER_IS_NODELET
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::CanNodelet, nodelet::Nodelet)
#endif
