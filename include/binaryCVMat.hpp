#pragma once
#include <opencv2/core/core.hpp>
#include <fstream>


bool writeMatBinary(std::ofstream& ofs, const cv::Mat& out_mat);
bool SaveMatBinary(const std::string& filename, const cv::Mat& output);
bool readMatBinary(std::ifstream& ifs, cv::Mat& in_mat);
bool LoadMatBinary(const std::string& filename, cv::Mat& output);