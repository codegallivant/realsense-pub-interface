#include <librealsense2/rs.hpp>
#include <zmq.hpp>
#include <opencv2/opencv.hpp>
#include <zstd.h>
#include <lz4.h>
#include <chrono>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <filesystem>
#include <atomic>  
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <iomanip>
#include <sstream>
#include <algorithm>

namespace fs = std::filesystem;

// Configuration
constexpr int WIDTH = 848, HEIGHT = 480, FPS = 10;
constexpr int BATCH_SIZE = 30; // 3 seconds of data per file
constexpr int ZSTD_LEVEL = 1;  // Fast compression
constexpr int LZ4_ACCEL = 10;  // Max acceleration

struct GpsData {
    double latitude;
    double longitude;
    double altitude;
};

class SignalManager {
public:
    SignalManager(int timeout = 2000):
        context(1),
        socket(context, ZMQ_REP),
        signal(false) 
    {
        socket.bind("tcp://*:5554");
        socket.set(zmq::sockopt::rcvtimeo, timeout);
    }

    void start(std::function<void(void)> start_callback, std::function<void(void)> stop_callback) {
        std::cout << "Waiting for signals..." << std::endl;
        while (true) {
            zmq::message_t request;
            if (socket.recv(request, zmq::recv_flags::none)) {
                std::string request_str(static_cast<char*>(request.data()), request.size());
                if (request_str == "start") {
                    if(signal == false) {
                        std::cout << "Starting.." << std::endl;
                        start_callback();
                    }
                    signal = true;
                } else if (request_str == "stop") {
                    if(signal == true) {
                        std::cout << "Stopping.." << std::endl;
                        stop_callback();
                    }
                    signal = false;
                } 
                std::string reply_str = "ack";
                zmq::message_t reply(reply_str.size());
                memcpy(reply.data(), reply_str.data(), reply_str.size());
                socket.send(reply, zmq::send_flags::none);
            }
        }
    }

private:
    std::atomic<bool> signal;
    zmq::context_t context;
    zmq::socket_t socket;
    int timeout;
};

class RealSenseProcessor : public rclcpp::Node {
public:
    RealSenseProcessor() 
        : Node("realsense_processor"),
          pipe(),
          cfg(),
          align(RS2_STREAM_COLOR),
          ctx(1),
          depth_socket(ctx, ZMQ_PUB),
          rgb_socket(ctx, ZMQ_PUB),
          running(false),
          current_gps_({0.0, 0.0, 0.0})
    {
        // RealSense configuration
        cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, 30);
        pipe.start(cfg);

        // ZeroMQ setup
        depth_socket.bind("tcp://*:5555");
        rgb_socket.bind("tcp://*:5557");

        // ROS2 GPS subscriber
        gps_subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps_cube", 10,
            std::bind(&RealSenseProcessor::gps_callback, this, std::placeholders::_1));
    }

    void start() {
        running = true;
        writer_thread = std::thread(&RealSenseProcessor::file_writer, this);
        processing_thread = std::thread(&RealSenseProcessor::processing_loop, this);
    }

    void stop() {
        running = false;
        if(processing_thread.joinable()) processing_thread.join();
        if(writer_thread.joinable()) writer_thread.join();
    }

    void close_pipe() {
        pipe.stop();
    }

private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(gps_mutex_);
        current_gps_.latitude = msg->latitude;
        current_gps_.longitude = msg->longitude;
        current_gps_.altitude = msg->altitude;
    }

    void processing_loop() {
        auto next_frame = std::chrono::steady_clock::now();

        while(running) {
            auto frames = pipe.wait_for_frames();
            auto aligned = align.process(frames);
            
            // Get current GPS data
            GpsData gps_data;
            {
                std::lock_guard<std::mutex> lock(gps_mutex_);
                gps_data = current_gps_;
            }
            
            // Process depth
            auto depth = aligned.get_depth_frame();
            auto depth_data = compress_depth(depth);
            
            // Process RGB
            auto color = aligned.get_color_frame();
            auto rgb_data = compress_rgb(color);

            // Network send with GPS data
            send_zmq_with_gps(depth_socket, depth_data, gps_data);
            send_zmq_with_gps(rgb_socket, rgb_data, gps_data);

            // Queue for file writing
            {
                std::lock_guard<std::mutex> lock(queue_mutex);
                depth_queue.push({depth_data, gps_data});
                rgb_queue.push({rgb_data, gps_data});
            }

            // Maintain FPS rate
            next_frame += std::chrono::milliseconds(1000/FPS);
            std::this_thread::sleep_until(next_frame);
            
            rclcpp::spin_some(this->get_node_base_interface());
        }
    }

    std::vector<uint8_t> compress_depth(const rs2::depth_frame& frame) {
        size_t bound = ZSTD_compressBound(frame.get_data_size());
        std::vector<uint8_t> buffer(bound);
        size_t size = ZSTD_compress(buffer.data(), bound,
                                  frame.get_data(), frame.get_data_size(), 
                                  ZSTD_LEVEL);
        buffer.resize(size);
        return buffer;
    }

    std::vector<uint8_t> compress_rgb(const rs2::video_frame& frame) {
        std::vector<uint8_t> buffer;
        cv::Mat cv_frame(cv::Size(WIDTH, HEIGHT), CV_8UC3, 
                        (void*)frame.get_data(), cv::Mat::AUTO_STEP);
        cv::imencode(".webp", cv_frame, buffer, {cv::IMWRITE_WEBP_QUALITY, 100});
        return buffer;
    }

    void send_zmq_with_gps(zmq::socket_t& socket, const std::vector<uint8_t>& data, const GpsData& gps) {
        size_t total_size = 3 * sizeof(double) + data.size();
        zmq::message_t msg(total_size);
        
        char* ptr = static_cast<char*>(msg.data());
        memcpy(ptr, &gps.latitude, sizeof(double));
        ptr += sizeof(double);
        memcpy(ptr, &gps.longitude, sizeof(double));
        ptr += sizeof(double);
        memcpy(ptr, &gps.altitude, sizeof(double));
        ptr += sizeof(double);
        memcpy(ptr, data.data(), data.size());
        
        socket.send(msg, zmq::send_flags::dontwait);
    }

    std::string format_gps_filename(const std::string& prefix, size_t counter, const GpsData& gps) {
        std::stringstream ss;
        
        std::string lat_str = std::to_string(gps.latitude);
        std::string lon_str = std::to_string(gps.longitude);
        std::string alt_str = std::to_string(gps.altitude);
        
        std::replace(lat_str.begin(), lat_str.end(), '.', '_');
        std::replace(lon_str.begin(), lon_str.end(), '.', '_');
        std::replace(alt_str.begin(), alt_str.end(), '.', '_');
        
        ss << prefix << counter << "__"
           << lat_str << "__"
           << lon_str << "__"
           << alt_str
           << ".bin";
           
        return ss.str();
    }

    void file_writer() {
        std::vector<std::pair<std::vector<uint8_t>, GpsData>> depth_batch, rgb_batch;
        size_t file_counter = 0;

        while(running || !depth_queue.empty() || !rgb_queue.empty()) {
            // Batch depth frames
            {
                std::lock_guard<std::mutex> lock(queue_mutex);
                while(!depth_queue.empty() && depth_batch.size() < BATCH_SIZE) {
                    depth_batch.push_back(std::move(depth_queue.front()));
                    depth_queue.pop();
                }
            }

            // Batch RGB frames
            {
                std::lock_guard<std::mutex> lock(queue_mutex);
                while(!rgb_queue.empty() && rgb_batch.size() < BATCH_SIZE) {
                    rgb_batch.push_back(std::move(rgb_queue.front()));
                    rgb_queue.pop();
                }
            }

            // Write batches
            if(!depth_batch.empty()) {
                write_batch("frames/depth_", file_counter, depth_batch);
                depth_batch.clear();
            }
            if(!rgb_batch.empty()) {
                write_batch("frames/rgb_", file_counter, rgb_batch);
                rgb_batch.clear();
                file_counter++;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void write_batch(const std::string& prefix, size_t counter,
                    const std::vector<std::pair<std::vector<uint8_t>, GpsData>>& batch) {
        if(batch.empty()) return;
        
        std::string filename = format_gps_filename(prefix, counter, batch[0].second);
        std::ofstream file(filename, std::ios::binary | std::ios::trunc);
        
        for(const auto& [data, gps] : batch) {
            // Write GPS data
            file.write(reinterpret_cast<const char*>(&gps.latitude), sizeof(double));
            file.write(reinterpret_cast<const char*>(&gps.longitude), sizeof(double));
            file.write(reinterpret_cast<const char*>(&gps.altitude), sizeof(double));
            
            // Write frame data size and content
            uint32_t size = data.size();
            file.write(reinterpret_cast<const char*>(&size), sizeof(size));
            file.write(reinterpret_cast<const char*>(data.data()), size);
        }
    }

private:
    // RealSense members
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::align align;
    
    // ZMQ members
    zmq::context_t ctx;
    zmq::socket_t depth_socket;
    zmq::socket_t rgb_socket;
    
    // Threading members
    std::atomic<bool> running;
    std::thread processing_thread;
    std::thread writer_thread;
    
    // Queue members
    std::mutex queue_mutex;
    std::queue<std::pair<std::vector<uint8_t>, GpsData>> depth_queue;
    std::queue<std::pair<std::vector<uint8_t>, GpsData>> rgb_queue;
    
    // GPS members
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_subscription_;
    std::mutex gps_mutex_;
    GpsData current_gps_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto processor = std::make_shared<RealSenseProcessor>();
    
    SignalManager signal_manager;
    auto start_callback = [&]() { processor->start(); };
    auto stop_callback = [&]() { processor->stop(); };
    
    signal_manager.start(start_callback, stop_callback);
    
    processor->close_pipe();
    rclcpp::shutdown();
    return 0;
}