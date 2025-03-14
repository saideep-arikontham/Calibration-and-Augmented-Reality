// =========================================================================================================
// Saideep Arikontham
// February 2025
// CS 5330 Project 3
// =========================================================================================================


#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <opencv2/opencv.hpp>
#include "utils.h"
#include <algorithm>
#include <fstream>


// =========================================================================================================
// Functions to find and draw chess board corners
// =========================================================================================================

bool find_checkerboard_corners(cv::Mat &gray, cv::Size &patternSize, std::vector<cv::Point2f> &corners){
    bool patternFound = cv::findChessboardCorners(
        gray, 
        patternSize, 
        corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | 
        cv::CALIB_CB_NORMALIZE_IMAGE
    );

    return patternFound;
}

int draw_checkerboard_corners(cv::Mat &frame, cv::Mat &gray, cv::Size &patternSize, std::vector<cv::Point2f> &corners, bool patternFound, cv::TermCriteria termcrit){
    if (patternFound) {
        // Draw the corners on the frame
        cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1), termcrit);
        cv::drawChessboardCorners(frame, patternSize, corners, patternFound);
    }
    return 0;
}


bool find_and_draw_checkerboard_corners(cv::Mat &frame, cv::Mat &dst, cv::Size &patternSize, std::vector<cv::Point2f> &corners, cv::TermCriteria termcrit){
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    frame.copyTo(dst);
    // Store the corners
    // std::vector<cv::Point2f> corners;
    
    // Find and draw
    bool patternFound = find_checkerboard_corners(gray, patternSize, corners);
    
    draw_checkerboard_corners(dst, gray, patternSize, corners, patternFound, termcrit);

    return patternFound;
}


// =========================================================================================================
// Functions to calibrate the camera
// =========================================================================================================


int write_results(const std::string &feature_file_path, cv::Mat camera_matrix, cv::Mat dist_coeffs) {

    FILE *featureFile = fopen(feature_file_path.c_str(), "w");
    if (!featureFile) {
        std::cerr << "Error opening file: " << feature_file_path << " for saving features.\n";
        return -1;
    }

    // Write camera_matrix values to the file
    for (int i = 0; i < camera_matrix.rows; i++) {
        for (int j = 0; j < camera_matrix.cols; j++) {
            fprintf(featureFile, "%.6f,", camera_matrix.at<double>(i, j));
        }
    }

    fprintf(featureFile, "\n");

    // Write dist_coeffs values to the file
    for (int i = 0; i < dist_coeffs.rows; i++) {
        for (int j = 0; j < dist_coeffs.cols; j++) {
            fprintf(featureFile, "%.6f,", dist_coeffs.at<double>(i, j));
        }
    }

    fclose(featureFile);
    return 0;
}


double perform_calibration(const std::vector<std::vector<cv::Point2f>>& corner_list, const std::vector<std::vector<cv::Vec3f>>& point_list, const cv::Size& refS, const cv::Mat& frame, cv::TermCriteria termcrit) {

    // Camera matrix initialization
    cv::Mat camera_matrix = cv::Mat::zeros(3, 3, CV_64F);
    camera_matrix.at<double>(0, 0) = 1;
    camera_matrix.at<double>(1, 1) = 1;
    camera_matrix.at<double>(0, 2) = frame.cols / 2;
    camera_matrix.at<double>(1, 2) = frame.rows / 2;

    cv::Mat dist_coeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    double reproj_error = cv::calibrateCamera(point_list, corner_list, refS, camera_matrix, dist_coeffs, 
                                                rvecs, tvecs, cv::CALIB_FIX_ASPECT_RATIO, termcrit);

    std::cout << "Initial Camera Matrix:\n" << camera_matrix << std::endl;
    std::cout << "Distortion Coefficients:\n" << dist_coeffs << std::endl;
    printf("Reprojection Error: %f\n", reproj_error);

    // Write calibration results to a file
    write_results("./files/calibration_results.csv", camera_matrix, dist_coeffs);


    return reproj_error;
}


// =========================================================================================================
// Functions to read calibration results
// =========================================================================================================


int read_calibration_results(const std::string &file_path, cv::Mat &camera_matrix, cv::Mat &dist_coeffs) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Error opening calibration file!" << std::endl;
        return -1;
    }
    
    camera_matrix = cv::Mat(3, 3, CV_64F);
    dist_coeffs = cv::Mat(1, 5, CV_64F);
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            file >> camera_matrix.at<double>(i, j);
            file.ignore(1, ',');
        }
    }
    for (int i = 0; i < 5; i++) {
        file >> dist_coeffs.at<double>(0, i);
        file.ignore(1, ',');
    }
    file.close();
    return 0;
}


// =========================================================================================================
// Functions to get virtual object data
// =========================================================================================================


void draw_projected_axes(cv::Mat &frame, const std::vector<cv::Vec3f> &axes_points, const cv::Mat &rvec, const cv::Mat &tvec, const cv::Mat &camera_matrix) {
    std::vector<cv::Point2f> projected_axes;
    cv::projectPoints(axes_points, rvec, tvec, camera_matrix, cv::Mat(), projected_axes);

    // Draw X, Y, and Z axes
    cv::line(frame, projected_axes[0], projected_axes[1], cv::Scalar(0, 0, 255), 3);  // X-axis (Red)
    cv::line(frame, projected_axes[0], projected_axes[2], cv::Scalar(0, 255, 0), 3);  // Y-axis (Green)
    cv::line(frame, projected_axes[0], projected_axes[3], cv::Scalar(255, 0, 0), 3);  // Z-axis (Blue)
}


void get_pyramid_object(std::vector<cv::Vec3f> &object_points, std::vector<std::pair<int, int>> &edges) {
    // Define 3D vertices of an asymmetrical pyramid floating above the board
    object_points = {
        {0, 0, 5},    // Apex
        {-1, -1, 1},  // Base corner 1
        {1, -1, 1},   // Base corner 2
        {1, 1, 1},    // Base corner 3
        {-1, 1, 1}    // Base corner 4
    };

    // Define edges connecting the vertices
    edges = {
        {0, 1}, {0, 2}, {0, 3}, {0, 4},  // Connect apex to base
        {1, 2}, {2, 3}, {3, 4}, {4, 1}   // Connect base edges
    };
}


void get_house_object(std::vector<cv::Vec3f> &object_points, std::vector<std::pair<int, int>> &edges) {
    // Define 3D vertices of a house - smaller scale and centered
    float scale = 0.6; // Reduced scale factor
    float height_offset = 0.5; // Lift it a bit above the board
    
    object_points = {
        {-2*scale, -2*scale, height_offset},  // 0: base front left
        {2*scale, -2*scale, height_offset},   // 1: base front right
        {2*scale, 2*scale, height_offset},    // 2: base back right
        {-2*scale, 2*scale, height_offset},   // 3: base back left
        {-2*scale, -2*scale, 3*scale+height_offset},  // 4: roof front left
        {2*scale, -2*scale, 3*scale+height_offset},   // 5: roof front right
        {2*scale, 2*scale, 3*scale+height_offset},    // 6: roof back right
        {-2*scale, 2*scale, 3*scale+height_offset},   // 7: roof back left
        {0, -2*scale, 5*scale+height_offset},   // 8: front roof peak
        {0, 2*scale, 5*scale+height_offset}     // 9: back roof peak
    };

    float centerX = 4.0;
    float centerY = -2.0;
    
    // Translate all points to center them on the board
    for (auto &pt : object_points) {
        // Apply translation to center the object
        pt[0] += centerX;
        pt[1] += centerY;
        pt[2] += 1;
    }

    // Define edges connecting the vertices
    edges = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0},  // Base
        {4, 5}, {5, 6}, {6, 7}, {7, 4},  // Roof base
        {0, 4}, {1, 5}, {2, 6}, {3, 7},  // Connecting vertical edges
        {4, 8}, {5, 8}, {6, 9}, {7, 9},  // Roof slopes
        {8, 9}                           // Roof ridge
    };
}


void get_s_letter_object1(std::vector<cv::Vec3f> &object_points, std::vector<std::pair<int, int>> &edges) {
    // Define 3D vertices of an "S" letter shape with more detail
    object_points = {
        // Top curve points (front face, z=0)
        {1.0, 1.5, 0},     // 0: top right
        {0.5, 1.7, 0},     // 1: top right curve
        {0.0, 1.8, 0},     // 2: top middle
        {-0.5, 1.7, 0},    // 3: top left curve
        {-1.0, 1.5, 0},    // 4: top left
        {-1.2, 1.2, 0},    // 5: upper left side
        {-1.3, 0.9, 0},    // 6: upper left curve
        {-1.2, 0.6, 0},    // 7: middle left
        
        // Middle curve points
        {-0.8, 0.3, 0},    // 8: middle left curve
        {-0.3, 0.1, 0},    // 9: middle upper
        {0.0, 0.0, 0},     // 10: center point
        {0.3, -0.1, 0},    // 11: middle lower
        {0.8, -0.3, 0},    // 12: middle right curve
        
        // Bottom curve points
        {1.2, -0.6, 0},    // 13: middle right
        {1.3, -0.9, 0},    // 14: lower right curve
        {1.2, -1.2, 0},    // 15: lower right side
        {1.0, -1.5, 0},    // 16: bottom right
        {0.5, -1.7, 0},    // 17: bottom right curve
        {0.0, -1.8, 0},    // 18: bottom middle
        {-0.5, -1.7, 0},   // 19: bottom left curve
        {-1.0, -1.5, 0},   // 20: bottom left
        
        // Back face (z=0.8) - same points with depth
        {1.0, 1.5, 0.8},   // 21
        {0.5, 1.7, 0.8},   // 22
        {0.0, 1.8, 0.8},   // 23
        {-0.5, 1.7, 0.8},  // 24
        {-1.0, 1.5, 0.8},  // 25
        {-1.2, 1.2, 0.8},  // 26
        {-1.3, 0.9, 0.8},  // 27
        {-1.2, 0.6, 0.8},  // 28
        {-0.8, 0.3, 0.8},  // 29
        {-0.3, 0.1, 0.8},  // 30
        {0.0, 0.0, 0.8},   // 31
        {0.3, -0.1, 0.8},  // 32
        {0.8, -0.3, 0.8},  // 33
        {1.2, -0.6, 0.8},  // 34
        {1.3, -0.9, 0.8},  // 35
        {1.2, -1.2, 0.8},  // 36
        {1.0, -1.5, 0.8},  // 37
        {0.5, -1.7, 0.8},  // 38
        {0.0, -1.8, 0.8},  // 39
        {-0.5, -1.7, 0.8}, // 40
        {-1.0, -1.5, 0.8}  // 41
    };

    float x = 7.0; // Start from center of checkerboard
    float y = -5.0;

    // Translate all points to center them on the board
    for (auto &pt : object_points) {
        // Apply translation to center the object
        pt[0] += x;
        pt[1] += y;
        pt[1] += 2;
    }

    // Define edges connecting the vertices
    edges = {
        // Connect front face (z=0) - outline of the "S"
        {0, 1}, {1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6}, {6, 7},
        {7, 8}, {8, 9}, {9, 10}, {10, 11}, {11, 12}, {12, 13},
        {13, 14}, {14, 15}, {15, 16}, {16, 17}, {17, 18}, {18, 19}, {19, 20},
        
        // Connect back face (z=0.8) - outline of the "S"
        {21, 22}, {22, 23}, {23, 24}, {24, 25}, {25, 26}, {26, 27}, {27, 28},
        {28, 29}, {29, 30}, {30, 31}, {31, 32}, {32, 33}, {33, 34},
        {34, 35}, {35, 36}, {36, 37}, {37, 38}, {38, 39}, {39, 40}, {40, 41},
        
        // Connect front to back - vertical supports at key points
        {0, 21}, {4, 25}, {7, 28}, {10, 31}, {13, 34}, {16, 37}, {20, 41},
        
        // Additional internal structural edges for complexity
        {2, 23}, {9, 30}, {11, 32}, {18, 39},
        
        // Cross braces for added complexity
        {0, 25}, {4, 21}, {7, 34}, {13, 28}, {16, 41}, {20, 37}
    };
}


// =========================================================================================================
// Functions to draw virtual object
// =========================================================================================================


void draw_virtual_object(cv::Mat &frame, const cv::Mat &rvec, const cv::Mat &tvec, const cv::Mat &camera_matrix, void (*get_object_func)(std::vector<cv::Vec3f>&, std::vector<std::pair<int, int>>&), const cv::Scalar &color) {
    // Define the virtual object points and edges
    std::vector<cv::Vec3f> object_points;
    std::vector<std::pair<int, int>> edges;
    get_object_func(object_points, edges);

    // Project points to 2D
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix, cv::Mat(), projected_points);

    // Draw edges with specified color
    for (const auto& edge : edges) {
        cv::line(frame, projected_points[edge.first], projected_points[edge.second], color, 8);
    }
}


// =========================================================================================================
// Extensions
// =========================================================================================================


void get_filled_pyramid_object(std::vector<cv::Vec3f> &object_points, std::vector<std::pair<int, int>> &edges, std::vector<std::vector<int>> &faces) {
    // Define 3D vertices of the pyramid
    object_points = {
    {0, 0, 3},    // Apex
    {-1, -1, 0},  // Base corner 1
    {1, -1, 0},   // Base corner 2
    {1, 1, 0},    // Base corner 3
    {-1, 1, 0}    // Base corner 4
    };

    // Define edges (for wireframe)
    edges = {
    {0, 1}, {0, 2}, {0, 3}, {0, 4},  // Apex to base
    {1, 2}, {2, 3}, {3, 4}, {4, 1}   // Base edges
    };

    static float x = 4.0;
    static float y = -2.0;

    static float dx = 0.2; // Initial movement speed in x
    static float dy = 0.2; // Initial movement speed in y
    
    // Define checkerboard boundaries
    float x_min = 1.0, x_max = 7.0;
    float y_min = -4.0, y_max = -1.0;
    
    // Update position
    x += dx;
    y += dy;
    
    // If it hits a boundary, bounce back with a slight random variation
    if (x <= x_min || x >= x_max) {
        dx = -dx * (0.8 + static_cast<float>(rand()) / RAND_MAX * 0.4);  // Random variation (0.8 to 1.2)
        x = std::max(x_min, std::min(x, x_max));  // Keep within bounds
    }
    if (y <= y_min || y >= y_max) {
        dy = -dy * (0.8 + static_cast<float>(rand()) / RAND_MAX * 0.4);  // Random variation (0.8 to 1.2)
        y = std::max(y_min, std::min(y, y_max));  // Keep within bounds
    }

    // Translate all points to center them on the board
    for (auto &pt : object_points) {
        // Apply translation to center the object
        pt[0] += x;
        pt[1] += y;
    }

    // Define faces (triangles & quadrilateral)
    faces = {
    {0, 1, 2},  // Side 1
    {0, 2, 3},  // Side 2
    {0, 3, 4},  // Side 3
    {0, 4, 1},  // Side 4
    {1, 2, 3, 4} // Base (quadrilateral)
    };
}


void get_filled_s_letter(std::vector<cv::Vec3f> &object_points, 
                          std::vector<std::pair<int, int>> &edges, 
                          std::vector<std::vector<int>> &faces) {
    // Define 3D vertices of the S letter
    object_points = {
        {0, 0, 1}, {3, 0, 1}, {0, -2.5, 1}, {3, -1, 1},
        {0, -3, 1}, {3, -1.5, 1}, {3, -4, 1}, {0, -4, 1},
        {1, -1, 1}, {3, -1, 1}, {2, -3, 1}, {2, -2.5, 1}, {1, -1.5, 1},

        {0, 0, 2}, {3, 0, 2}, {0, -2.5, 2}, {3, -1, 2},
        {0, -3, 2}, {3, -1.5, 2}, {3, -4, 2}, {0, -4, 2},
        {1, -1, 2}, {3, -1, 2}, {2, -3, 2}, {2, -2.5, 2}, {1, -1.5, 2}
    };

    edges = {
        {0, 1}, {1, 3}, {0, 2}, {6, 7}, {5, 6}, {4, 7}, {8,9}, {3, 9}, 
        {4, 10}, {2, 11}, {10, 11}, {8, 12}, {5, 12},
        {13, 14}, {14, 16}, {13, 15}, {19, 20}, {18, 19}, {17, 20}, {21, 22}, 
        {16, 22}, {17, 23}, {15, 24}, {23, 24}, {21, 25}, {18, 25},
        {0, 13}, {1, 14}, {2, 15}, {3, 16}, {4, 17}, {5, 18}, {6, 19}, {7, 20}, 
        {8, 21}, {9, 22}, {10, 23}, {11, 24}, {12, 25}
    };

    faces = {
        {0, 1, 3, 2},   // Top face of the S
        {4, 5, 6, 7},   // Bottom face
        {8, 9, 3, 2, 11, 10},  // Middle bridge (properly connected)
        {12, 8, 10, 5}, // Inner bridge

        {13, 14, 16, 15},  // Top back face
        {17, 18, 19, 20},  // Bottom back face
        {21, 22, 16, 15, 24, 23},  // Middle bridge back
        {25, 21, 23, 18}, // Inner bridge back

        // Side faces (connecting front to back)
        {0, 13, 15, 2},
        {1, 14, 16, 3},
        {4, 17, 20, 7},
        {5, 18, 19, 6},
        {8, 21, 22, 9},
        {10, 23, 24, 11},
        {12, 25, 21, 8}
    };
}






void draw_filled_virtual_object(cv::Mat &frame, const cv::Mat &rvec, const cv::Mat &tvec, const cv::Mat &camera_matrix, void (*get_object_func)(std::vector<cv::Vec3f>&, std::vector<std::pair<int, int>>&, std::vector<std::vector<int>>&), const cv::Scalar &color) {
    std::vector<cv::Vec3f> object_points;
    std::vector<std::pair<int, int>> edges;
    std::vector<std::vector<int>> faces;

    get_object_func(object_points, edges, faces);

    // Project 3D points to 2D
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix, cv::Mat(), projected_points);

    // Fill faces first
    for (size_t i = 0; i < faces.size(); ++i) {
        std::vector<cv::Point> polygon;
        for (size_t j = 0; j < faces[i].size(); ++j) {
            int index = faces[i][j];
            polygon.push_back(projected_points[index]);
        }
        cv::fillConvexPoly(frame, polygon, color);
    }

    // Draw edges on top to outline shape
    for (size_t i = 0; i < edges.size(); ++i) {
        int idx1 = edges[i].first;
        int idx2 = edges[i].second;
        cv::line(frame, projected_points[idx1], projected_points[idx2], cv::Scalar(0, 0, 0), 2);
    }

}


void replace_checkerboard_with_image(cv::Mat &frame, const std::vector<cv::Point2f> &corner_set, const std::vector<cv::Vec3f> &point_set, const cv::Size &pattern_size, const cv::Mat &overlay_img, const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs,const cv::Mat &rvec, const cv::Mat &tvec) {
    // Project the 3D world coordinates to 2D image points
    std::vector<cv::Point2f> projected_points;
    cv::projectPoints(point_set, rvec, tvec, camera_matrix, dist_coeffs, projected_points);

    if (projected_points.size() != pattern_size.area()) {
        std::cerr << "Error: Incorrect number of projected points.\n";
        return;
    }

    // Get the four outer corners in consistent order
    int width = pattern_size.width;
    int height = pattern_size.height;
    std::vector<cv::Point2f> checkerboard_corners = {
        projected_points[0],                       // top-left (0,0)
        projected_points[width-1],                 // top-right (width-1,0)
        projected_points[(height-1)*width + (width-1)], // bottom-right (width-1,height-1)
        projected_points[(height-1)*width]         // bottom-left (0,height-1)
    };

    // Define overlay image corners in same order (clockwise)
    std::vector<cv::Point2f> overlay_corners = {
        cv::Point2f(0, 0),                      // top-left
        cv::Point2f(overlay_img.cols - 1, 0),   // top-right
        cv::Point2f(overlay_img.cols - 1, overlay_img.rows - 1),  // bottom-right
        cv::Point2f(0, overlay_img.rows - 1)    // bottom-left
    };

    // Compute homography
    cv::Mat H = cv::findHomography(overlay_corners, checkerboard_corners);

    // Warp overlay image onto the frame
    cv::Mat warped_overlay;
    cv::warpPerspective(overlay_img, warped_overlay, H, frame.size());

    // Create mask from warped overlay
    cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);
    std::vector<cv::Point> checkerboard_corners_int;
    for (const auto &pt : checkerboard_corners) {
        checkerboard_corners_int.push_back(cv::Point(static_cast<int>(pt.x), static_cast<int>(pt.y)));
    }
    cv::fillConvexPoly(mask, checkerboard_corners_int, cv::Scalar(255));

    // Blend warped overlay onto the frame
    warped_overlay.copyTo(frame, mask);
}






