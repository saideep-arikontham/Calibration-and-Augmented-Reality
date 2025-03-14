#include <iostream>
#include <opencv2/opencv.hpp>

int main() {
    // Define pattern size (example values)
    cv::Size pattern_size(5, 4); // Width x Height

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

    // Display the point_set
    std::cout << "point_set contents:\n";
    for (const auto& point : point_set) {
        std::cout << "(" << point[0] << ", " << point[1] << ", " << point[2] << ")\n";
    }

    return 0;
}
