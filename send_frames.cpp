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


namespace fs = std::filesystem;

// Configuration
constexpr int WIDTH = 640, HEIGHT = 480, FPS = 15;
constexpr int BATCH_SIZE = 30; // 3 seconds of data per file
constexpr int ZSTD_LEVEL = 1;  // Fast compression
constexpr int LZ4_ACCEL = 10;  // Max acceleration

class RealSenseProcessor {
public:
    RealSenseProcessor() 
        : align(RS2_STREAM_COLOR),
          ctx(1),
          depth_socket(ctx, ZMQ_PUB),
          rgb_socket(ctx, ZMQ_PUB),
          running(false)
    {
        // RealSense configuration
        cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, FPS);
        cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, FPS);
        pipe.start(cfg);

        // ZeroMQ setup
        depth_socket.bind("tcp://*:5555");
        rgb_socket.bind("tcp://*:5557");
    }

    void start() {
        running = true;
        writer_thread = std::thread(&RealSenseProcessor::file_writer, this);
        processing_thread = std::thread(&RealSenseProcessor::processing_loop, this);
    }

    void stop() {
        running = false;
        processing_thread.join();
        writer_thread.join();
        pipe.stop();
    }

private:
    void processing_loop() {
        rs2::align align(RS2_STREAM_COLOR);
        auto next_frame = std::chrono::steady_clock::now();

        while(running) {
            auto start = std::chrono::steady_clock::now();

            // Capture frame
            auto frames = pipe.wait_for_frames();
            auto aligned = align.process(frames);
            
            // Process depth
            auto depth = aligned.get_depth_frame();
            auto depth_data = compress_depth(depth);
            
            // Process RGB
            auto color = aligned.get_color_frame();
            auto rgb_data = compress_rgb(color);

            // Network send
            send_zmq(depth_socket, depth_data);
            send_zmq(rgb_socket, rgb_data);

            // Queue for file writing
            {
                std::lock_guard<std::mutex> lock(queue_mutex);
                depth_queue.push(depth_data);
                rgb_queue.push(rgb_data);
            }

            // Maintain 10Hz rate
            next_frame += std::chrono::milliseconds(1000/FPS);
            std::this_thread::sleep_until(next_frame);
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

    void send_zmq(zmq::socket_t& socket, const std::vector<uint8_t>& data) {
        zmq::message_t msg(data.size());
        memcpy(msg.data(), data.data(), data.size());
        socket.send(msg, zmq::send_flags::dontwait);
    }

    void file_writer() {
        std::vector<std::vector<uint8_t>> depth_batch, rgb_batch;
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
                    const std::vector<std::vector<uint8_t>>& batch) {
        std::ofstream file(prefix + std::to_string(counter) + ".bin", 
                         std::ios::binary | std::ios::trunc);
        
        for(const auto& data : batch) {
            uint32_t size = data.size();
            file.write(reinterpret_cast<const char*>(&size), sizeof(size));
            file.write(reinterpret_cast<const char*>(data.data()), size);
        }
    }

    // Decompression functions
public:
    static std::vector<cv::Mat> decompress_depth_file(const std::string& filename) {
        std::ifstream file(filename, std::ios::binary);
        std::vector<cv::Mat> frames;
        
        while(file) {
            uint32_t size;
            file.read(reinterpret_cast<char*>(&size), sizeof(size));
            
            std::vector<uint8_t> buffer(size);
            file.read(reinterpret_cast<char*>(buffer.data()), size);
            
            cv::Mat frame(HEIGHT, WIDTH, CV_16UC1);
            ZSTD_decompress(frame.data, WIDTH*HEIGHT*2, 
                          buffer.data(), buffer.size());
            frames.push_back(frame);
        }
        
        return frames;
    }

    static std::vector<cv::Mat> decompress_rgb_file(const std::string& filename) {
        std::ifstream file(filename, std::ios::binary);
        std::vector<cv::Mat> frames;
        
        while(file) {
            uint32_t size;
            file.read(reinterpret_cast<char*>(&size), sizeof(size));
            
            std::vector<uint8_t> buffer(size);
            file.read(reinterpret_cast<char*>(buffer.data()), size);
            
            cv::Mat frame = cv::imdecode(buffer, cv::IMREAD_UNCHANGED);
            frames.push_back(frame);
        }
        
        return frames;
    }

    // Member variables
private:
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::align align;
    
    zmq::context_t ctx;
    zmq::socket_t depth_socket;
    zmq::socket_t rgb_socket;
    
    std::atomic<bool> running;
    std::thread processing_thread;
    std::thread writer_thread;
    
    std::mutex queue_mutex;
    std::queue<std::vector<uint8_t>> depth_queue;
    std::queue<std::vector<uint8_t>> rgb_queue;
};


int main() {
    RealSenseProcessor processor;
    processor.start();
    
    // Run for 1 minute
    std::this_thread::sleep_for(std::chrono::minutes(1));
    
    processor.stop();

    return 0;
}