#include <librealsense2/rs.hpp>
#include <zmq.hpp>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <thread>
#include "include/config.hpp"
#include "include/binaryCVMat.hpp"



cv::Mat rs2_to_cvmat(rs2::frame& frame, int h, int w, int type) {
    if (type == CV_16UC1) {
        cv::Mat mat(cv::Size(w,h), CV_16UC1, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
        return mat;
    } else if (type == CV_8UC3) {
        cv::Mat mat(cv::Size(w, h), CV_8UC3, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
        return mat;
    }
}

void save_mat(std::string filepath, cv::Mat& frameMat, int type) {
    if (type == CV_16UC1) {
        // save as binary data
        // cv::Mat frameMat(cv::Size(w, h), CV_16UC1, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
        SaveMatBinary(filepath.c_str(), frameMat);
    } else if(type == CV_8UC3) {
        // save as png
        // cv::Mat frameMat(cv::Size(w, h), CV_8UC3, (void*)frame.get_data(), cv::Mat::AUTO_STEP);
        cv::imwrite(filepath.c_str(), frameMat);
        // cv::imshow("some", frameMat);
        // cv::waitKey();
    }
}

void send_mat(zmq::socket_t& socket, const cv::Mat& mat) {
    // Metadata: rows, cols, type
    int metadata[3] = {mat.rows, mat.cols, mat.type()};
    zmq::message_t metadata_msg(sizeof(metadata));
    memcpy(metadata_msg.data(), metadata, sizeof(metadata));
    socket.send(metadata_msg, zmq::send_flags::sndmore);

    // Image data
    zmq::message_t image_msg(mat.total() * mat.elemSize());
    memcpy(image_msg.data(), mat.data, mat.total() * mat.elemSize());
    socket.send(image_msg, zmq::send_flags::none);
}


int main() {

    Configurator::Config config = Configurator::load_config("config.yaml");
  
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, config.width, config.height, RS2_FORMAT_BGR8, 30); 
    cfg.enable_stream(RS2_STREAM_DEPTH, config.width, config.height, RS2_FORMAT_Z16, 30);  
    rs2::pipeline_profile profile = pipe.start(cfg);

    zmq::context_t context(1);
    zmq::socket_t depth_socket(context, ZMQ_PUB);
    zmq::socket_t color_socket(context, ZMQ_PUB);
    depth_socket.bind("tcp://*:"+std::to_string(config.depth_port));
    color_socket.bind("tcp://*:"+std::to_string(config.color_port));

    int i = 0;
    while (true) {
        rs2::frameset frames = pipe.wait_for_frames();
    
        rs2::align alignTo(RS2_STREAM_COLOR);
        rs2::frameset aligned_frames = alignTo.process(frames);
        rs2::frame color_rs2 = aligned_frames.get_color_frame();
        rs2::frame depth_rs2 = aligned_frames.get_depth_frame();

        cv::Mat color = rs2_to_cvmat(color_rs2, config.height, config.width, CV_8UC3);
        cv::Mat depth = rs2_to_cvmat(depth_rs2, config.height, config.width, CV_16UC1);

        send_mat(color_socket, color);
        send_mat(depth_socket, depth);

        save_mat("frames/color_"+std::to_string(i)+".png", color, CV_8UC3);
        save_mat("frames/depth_"+std::to_string(i)+".bin", depth, CV_16UC1);

        std::this_thread::sleep_for(std::chrono::milliseconds(33));
        i++;
    }
   return 0;
}