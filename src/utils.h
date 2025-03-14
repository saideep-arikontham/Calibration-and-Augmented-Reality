// =========================================================================================================
// Saideep Arikontham
// February 2025
// CS 5330 Project 3
// =========================================================================================================


#ifndef UTILS_H
#define UTILS_H

#include <opencv2/opencv.hpp>

// Function prototype for object retrieval
typedef void (*GetObjectFunc)(std::vector<cv::Vec3f> &, std::vector<std::pair<int, int>> &);
typedef void (*GetFilledObjectFunc)(std::vector<cv::Vec3f> &, std::vector<std::pair<int, int>> &, std::vector<std::vector<int>> &);

// Functions to find and draw checkerboard corners
bool find_checkerboard_corners(cv::Mat &gray, cv::Size &patternSize, std::vector<cv::Point2f> &corners);
int draw_checkerboard_corners(cv::Mat &frame, cv::Mat &gray, cv::Size &patternSize, std::vector<cv::Point2f> &corners, bool patternFound, cv::TermCriteria termcrit);
bool find_and_draw_checkerboard_corners(cv::Mat &frame, cv::Mat &dst, cv::Size &patternSize, std::vector<cv::Point2f> &corners, cv::TermCriteria termcrit);

// Functions to calibrate camera
int write_results(const std::string &feature_file_path, cv::Mat camera_matrix, cv::Mat dist_coeffs);
double perform_calibration(const std::vector<std::vector<cv::Point2f>>& corner_list, const std::vector<std::vector<cv::Vec3f>>& point_list, const cv::Size& refS, const cv::Mat& frame, cv::TermCriteria termcrit);

// Function to read calibration results
int read_calibration_results(const std::string &file_path, cv::Mat &camera_matrix, cv::Mat &dist_coeffs);

// Function to get virtual object data
void draw_projected_axes(cv::Mat &frame, const std::vector<cv::Vec3f> &axes_points, const cv::Mat &rvec, const cv::Mat &tvec, const cv::Mat &camera_matrix);
void get_pyramid_object(std::vector<cv::Vec3f> &object_points, std::vector<std::pair<int, int>> &edges);
void get_house_object(std::vector<cv::Vec3f> &object_points, std::vector<std::pair<int, int>> &edges);
void get_s_letter_object1(std::vector<cv::Vec3f> &object_points, std::vector<std::pair<int, int>> &edges);
void get_ufo_object(std::vector<cv::Vec3f> &object_points, std::vector<std::pair<int, int>> &edges);
void draw_virtual_object(cv::Mat &frame, const cv::Mat &rvec, const cv::Mat &tvec, const cv::Mat &camera_matrix, void (*get_object_func)(std::vector<cv::Vec3f>&, std::vector<std::pair<int, int>>&), const cv::Scalar &color);

// Extensions
void get_filled_pyramid_object(std::vector<cv::Vec3f> &object_points, std::vector<std::pair<int, int>> &edges, std::vector<std::vector<int>> &faces);
void get_filled_s_letter(std::vector<cv::Vec3f> &object_points, std::vector<std::pair<int, int>> &edges, std::vector<std::vector<int>> &faces);
void draw_filled_virtual_object(cv::Mat &frame, const cv::Mat &rvec, const cv::Mat &tvec, const cv::Mat &camera_matrix, void (*get_object_func)(std::vector<cv::Vec3f>&, std::vector<std::pair<int, int>>&, std::vector<std::vector<int>>&), const cv::Scalar &color);

void replace_checkerboard_with_image(cv::Mat &frame, const std::vector<cv::Point2f> &corner_set, const std::vector<cv::Vec3f> &point_set, const cv::Size &pattern_size, const cv::Mat &overlay_img, const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs,const cv::Mat &rvec, const cv::Mat &tvec);

#endif
