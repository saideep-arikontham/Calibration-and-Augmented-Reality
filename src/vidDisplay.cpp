// ===================================================================================================
// Saideep Arikontham
// Mar 2025
// CS 5330 Project 4
// ===================================================================================================

#include <cstdio>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <algorithm>
#include "utils.h"

int main(int argc, char *argv[]) {
    cv::VideoCapture *capdev;
    capdev = new cv::VideoCapture(0);
    
    if (!capdev->isOpened()) {
        printf("Unable to open video device\n");
        return -1;
    }

    cv::Size refS((int)capdev->get(cv::CAP_PROP_FRAME_WIDTH),
                  (int)capdev->get(cv::CAP_PROP_FRAME_HEIGHT));
    printf("Expected size: %d %d\n", refS.width, refS.height);

    cv::namedWindow("Video", 1);
    cv::namedWindow("Detected Corners", 1);
    cv::Mat frame;

    // Define the pattern size as 9x6 internal corners
    cv::Size pattern_size(9, 6);

    // Load calibration parameters
    cv::Mat camera_matrix, dist_coeffs;
    read_calibration_results("./files/calibration_results.csv", camera_matrix, dist_coeffs);
    

    // Storage for calibration
    std::vector<std::vector<cv::Point2f>> corner_list;
    std::vector<std::vector<cv::Vec3f>> point_list;
    std::vector<cv::Vec3f> point_set;

    // Creating the 3D world coordinates, the coordinates would be same for every detection.
    // Only the corner set values would change.
    for (int i = 0; i < pattern_size.height; i++) {
        for (int j = 0; j < pattern_size.width; j++) {
            point_set.push_back(cv::Vec3f(j, -i, 0));  // 3D world coordinates
        }
    }

    cv::TermCriteria termcrit(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON);

    // Define 3D axes points at the checkerboard origin
    std::vector<cv::Vec3f> axes_points;
    axes_points.push_back(cv::Vec3f(0,0,0));  // origin
    axes_points.push_back(cv::Vec3f(3,0,0));  // X-axis (red)
    axes_points.push_back(cv::Vec3f(0,-3,0)); // Y-axis (green)
    axes_points.push_back(cv::Vec3f(0,0,-3)); // Z-axis (blue)

    char prev_key='o';
    

    for (;;) {
        *capdev >> frame;
        if (frame.empty()) {
            printf("Frame is empty\n");
            break;
        }

        // Store the corners
        std::vector<cv::Point2f> corner_set;
        bool found;

        // cv::resize(frame, frame, cv::Size(640, 480));
        cv::imshow("Video", frame);

        // Find and draw
        cv::Mat dst;
        found = find_and_draw_checkerboard_corners(frame, dst, pattern_size, corner_set, termcrit);
        cv::imshow("Detected Corners", dst);

        // Draw the 3D axes
        if(prev_key == 'p'){
            if(found){
                cv::Mat rvec, tvec;
                bool solved = cv::solvePnP(point_set, corner_set, camera_matrix, cv::Mat(), rvec, tvec);
                if (solved) {
                    std::cout << "Rotation Vector (rvec): " << rvec.t() << std::endl;
                    std::cout << "Translation Vector (tvec): " << tvec.t() << std::endl;
                    // Draw the 3D axes
                    draw_projected_axes(frame, axes_points, rvec, tvec, camera_matrix); 
                }
            }
            else{
                cv::putText(frame, "No corners detected", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
            }
            // Show with axes
            cv::imshow("3D Virtual Object", frame); 
        }

        // Draw Virtual Objects
        else if(prev_key == 't'){
            if(found){
                cv::Mat rvec, tvec;
                bool solved = cv::solvePnP(point_set, corner_set, camera_matrix, cv::Mat(), rvec, tvec);
        
                if (solved) {
                    draw_virtual_object(frame, rvec, tvec, camera_matrix, get_pyramid_object, cv::Scalar(255, 0, 0));
                    draw_virtual_object(frame, rvec, tvec, camera_matrix, get_house_object, cv::Scalar(0, 0, 255));
                    draw_virtual_object(frame, rvec, tvec, camera_matrix, get_s_letter_object1, cv::Scalar(0, 0, 255));
                }
            }
            else{
                cv::putText(frame, "No corners detected", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
            }
            // Show with axes
            cv::imshow("3D Virtual Object", frame); 
        }

        // Extensions
        else if(prev_key == 'f'){
            if(found){
                cv::Mat rvec, tvec;
                bool solved = cv::solvePnP(point_set, corner_set, camera_matrix, cv::Mat(), rvec, tvec);
        
                if (solved) {
                    draw_filled_virtual_object(frame, rvec, tvec, camera_matrix, get_filled_s_letter, cv::Scalar(0, 0, 255));
                }
            }
            else{
                cv::putText(frame, "No corners detected", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
            }
            // Show with axes
            cv::imshow("3D Virtual Object", frame); 
        }

        else if(prev_key == 'r'){
            if(found){
                cv::Mat rvec, tvec;
                bool solved = cv::solvePnP(point_set, corner_set, camera_matrix, dist_coeffs, rvec, tvec);
                
                if (solved) {
                    cv::Mat overlay_img = cv::imread("./images/sand.jpg");
                    replace_checkerboard_with_image(frame, corner_set, point_set, pattern_size, overlay_img, camera_matrix, dist_coeffs, rvec, tvec);
                    draw_filled_virtual_object(frame, rvec, tvec, camera_matrix, get_filled_pyramid_object, cv::Scalar(140, 180, 210));
                }
            } else {
                cv::putText(frame, "No corners detected", cv::Point(10, 30), 
                            cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
            }
            
            cv::imshow("3D Virtual Object", frame); 
        }

        char key = cv::waitKey(10);

        // Quit the program
        if (key == 'q') {
            break;
        } 

        // Save the frame with detected corners
        else if (key == 's' || key == 'S') {
            if(found){
                corner_list.push_back(corner_set);
                point_list.push_back(point_set);
                std::string img_name = "./images/calibration_image_" + std::to_string(corner_list.size()) + ".jpg";
                cv::imwrite(img_name, dst);
                printf("Saved calibration image with detected corners.\n");
            }
            else{
                printf("No corners detected. Choose different frame for calibration.\n");
            }
        }

        // Perform calibration with saved points
        else if (key == 'c' || key == 'C') {
            if (corner_list.size() >= 5) {
                std::vector<cv::Mat> rvecs, tvecs;
                double reproj_error = perform_calibration(corner_list, point_list, refS, frame, termcrit);
            } else {
                printf("Need at least 5 calibration frames.\n");
            }
        }

        // Track previous key pressed.
        if(key == 'p' || key == 'o' || key == 't' || key == 'f' || key == 'r'){
            prev_key = key;
        }
    }

    delete capdev;
    return 0;
}

