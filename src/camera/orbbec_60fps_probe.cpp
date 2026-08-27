// Gemini 336L dual-stream 60 FPS acceptance probe.
// Uses native OrbbecSDK profiles and device timestamps; it does not render or
// modify persistent camera properties.
#include <libobsensor/ObSensor.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace {

constexpr double kMinimumMeasuredFps = 59.4;

struct StreamStats {
    std::mutex mutex;
    uint64_t count = 0;
    uint64_t first_timestamp_us = 0;
    uint64_t last_timestamp_us = 0;
    uint64_t previous_timestamp_us = 0;
    uint64_t max_gap_us = 0;
    uint64_t nonmonotonic = 0;

    void add(uint64_t timestamp_us) {
        std::lock_guard<std::mutex> lock(mutex);
        if(first_timestamp_us == 0) {
            first_timestamp_us = timestamp_us;
        }
        if(previous_timestamp_us != 0) {
            if(timestamp_us <= previous_timestamp_us) {
                ++nonmonotonic;
            }
            else {
                max_gap_us = std::max(max_gap_us, timestamp_us - previous_timestamp_us);
            }
        }
        previous_timestamp_us = timestamp_us;
        last_timestamp_us = timestamp_us;
        ++count;
    }

    double measuredFps() const {
        if(count < 2 || last_timestamp_us <= first_timestamp_us) {
            return 0.0;
        }
        return static_cast<double>(count - 1) * 1000000.0
               / static_cast<double>(last_timestamp_us - first_timestamp_us);
    }
};

OBUvcBackendType parseBackend(const std::string &backend) {
    if(backend == "auto") {
        return OB_UVC_BACKEND_TYPE_AUTO;
    }
    if(backend == "libuvc") {
        return OB_UVC_BACKEND_TYPE_LIBUVC;
    }
    if(backend == "v4l2") {
        return OB_UVC_BACKEND_TYPE_V4L2;
    }
    throw std::invalid_argument("backend must be auto, v4l2, or libuvc");
}

}  // namespace

int main(int argc, char **argv) try {
    const std::string backend = argc > 1 ? argv[1] : "v4l2";
    const int duration_seconds = argc > 2 ? std::stoi(argv[2]) : 12;
    if(duration_seconds < 2) {
        throw std::invalid_argument("duration must be at least 2 seconds");
    }

    auto context = std::make_shared<ob::Context>();
    context->setUvcBackendType(parseBackend(backend));
    auto devices = context->queryDeviceList();
    if(devices->getCount() == 0) {
        std::cerr << "device_count=0\n";
        return 2;
    }

    auto device = devices->getDevice(0);
    auto info = device->getDeviceInfo();
    auto pipeline = std::make_shared<ob::Pipeline>(device);
    auto color = pipeline->getStreamProfileList(OB_SENSOR_COLOR)
                     ->getVideoStreamProfile(1280, 800, OB_FORMAT_MJPG, 60);
    auto depth = pipeline->getStreamProfileList(OB_SENSOR_DEPTH)
                     ->getVideoStreamProfile(848, 480, OB_FORMAT_Y16, 60);

    std::cout << "backend=" << backend << " device=" << info->getName()
              << " firmware=" << info->getFirmwareVersion()
              << " connection=" << info->getConnectionType() << '\n'
              << "requested_color=1280x800@60_MJPG\n"
              << "requested_depth=848x480@60_Y16\n";

    auto config = std::make_shared<ob::Config>();
    config->enableStream(color);
    config->enableStream(depth);
    config->setAlignMode(ALIGN_DISABLE);
    config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION);

    StreamStats color_stats;
    StreamStats depth_stats;
    pipeline->start(config, [&](std::shared_ptr<ob::FrameSet> frames) {
        if(!frames) {
            return;
        }
        if(auto color_frame = frames->getColorFrame()) {
            color_stats.add(color_frame->getTimeStampUs());
        }
        if(auto depth_frame = frames->getDepthFrame()) {
            depth_stats.add(depth_frame->getTimeStampUs());
        }
    });
    std::this_thread::sleep_for(std::chrono::seconds(duration_seconds));
    pipeline->stop();

    const double color_fps = color_stats.measuredFps();
    const double depth_fps = depth_stats.measuredFps();
    std::cout << std::fixed << std::setprecision(3)
              << "color_frames=" << color_stats.count << " actual_hz=" << color_fps
              << " max_gap_ms=" << color_stats.max_gap_us / 1000.0
              << " nonmonotonic=" << color_stats.nonmonotonic << '\n'
              << "depth_frames=" << depth_stats.count << " actual_hz=" << depth_fps
              << " max_gap_ms=" << depth_stats.max_gap_us / 1000.0
              << " nonmonotonic=" << depth_stats.nonmonotonic << '\n';

    const bool passed = color_fps >= kMinimumMeasuredFps
                        && depth_fps >= kMinimumMeasuredFps
                        && color_stats.nonmonotonic == 0
                        && depth_stats.nonmonotonic == 0;
    std::cout << "result=" << (passed ? "PASS" : "FAIL")
              << " threshold_hz=" << kMinimumMeasuredFps << '\n';
    return passed ? 0 : 3;
}
catch(const ob::Error &error) {
    std::cerr << "function=" << error.getFunction() << " args=" << error.getArgs()
              << " message=" << error.what() << " status=" << error.getStatus()
              << " type=" << error.getExceptionType() << '\n';
    return 1;
}
catch(const std::exception &error) {
    std::cerr << "error=" << error.what() << '\n';
    return 1;
}
