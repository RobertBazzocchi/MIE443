#pragma once

#include <vector>

#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <opencv2/core.hpp>
#include <cv.h>
#include <cv_bridge/cv_bridge.h>
#include <boxes.h>

// OPENCV
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/imgproc.hpp"

struct ImageFeatures
{
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> keypoints;
};

class ImagePipeline 
{
    public:
        ImagePipeline(ros::NodeHandle& n);

        void imageCallback(const sensor_msgs::ImageConstPtr& msg);

        int getTemplateID(Boxes& boxes);

        void populateTemplates(Boxes& boxes);

    private:
        cv::Mat img;
        bool isValid;
        image_transport::Subscriber sub;

        // SURF Matching
        std::vector<ImageFeatures> vTemplates;
        cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(600); // minHessian = 600
        cv::FlannBasedMatcher matcher;
};
