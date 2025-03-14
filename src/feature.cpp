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
#include <opencv2/xfeatures2d.hpp>

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
    cv::namedWindow("Corners", cv::WINDOW_AUTOSIZE);
    cv::Mat frame;
    char prev_key = 'a';

    for (;;) {
        *capdev >> frame;
        if (frame.empty()) {
            printf("Frame is empty\n");
            break;
        }

        cv::imshow("Video", frame);

        if(prev_key == 'h'){
            cv::Mat grey_scale, dst, dst_norm, dst_norm_scaled;
            // Convert frame to grayscale
            cv::cvtColor(frame, grey_scale, cv::COLOR_BGR2GRAY);

            int blockSize = 5;
            int apertureSize = 3;
            double k = 0.005;
            //cornerHarris function takes the grayscale image grey_scale, the output matrix dst, the block size, aperture size, and Harris parameter k.
            cornerHarris(grey_scale, dst, blockSize, apertureSize, k);
            //The corner response matrix is then normalized using normalize 
            normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
            //converted to an 8-bit representation using convertScaleAbs.
            convertScaleAbs(dst_norm, dst_norm_scaled);
            //iterates over each pixel of the normalized matrix
            for (int i = 0; i < dst_norm.rows; i++) {
              for (int j = 0; j < dst_norm.cols; j++) {
                //value above a threshold of 100, a red circle is drawn on the frame image using the circle function.
                if ((int) dst_norm.at<float>(i, j) > 100) {
                  circle(frame, cv::Point(j, i), 1, cv::Scalar(0, 0, 255), 1, 8, 0);
                }
              }
            }
            cv::putText(frame, "Harris Corners", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
            cv::imshow("Harris Corners", frame);
        }

        else if (prev_key == 's') {
          cv::Mat gray;
          cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
  
          // SURF feature detection
          int minHessian = 400;
          auto detector = cv::xfeatures2d::SURF::create(minHessian);
          std::vector<cv::KeyPoint> keypoints;
          detector->detect(gray, keypoints);
  
          // Draw keypoints
          cv::Mat frame_surf;
          cv::drawKeypoints(frame, keypoints, frame_surf, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DEFAULT);
  
          cv::putText(frame_surf, "SURF Features", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
          cv::imshow("SURF Features", frame_surf);
      }


        // Exit on 'q' key
        char key = cv::waitKey(10);
        if (key == 'q') break;


        if(key == 'h' || key == 's'){
          prev_key = key;
        }
    }

    delete capdev;
    return 0;
}

