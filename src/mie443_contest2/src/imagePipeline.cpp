#include <imagePipeline.h>

// OPENCV
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/imgproc.hpp"

#define IMAGE_TYPE sensor_msgs::image_encodings::BGR8
#define IMAGE_TOPIC "camera/image" // kinect:"camera/rgb/image_raw" webcam:"camera/image"

using namespace cv;
using namespace cv::xfeatures2d;

ImagePipeline::ImagePipeline(ros::NodeHandle& n) {
    image_transport::ImageTransport it(n);
    sub = it.subscribe(IMAGE_TOPIC, 1, &ImagePipeline::imageCallback, this);
    isValid = false;
}

void ImagePipeline::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        if(isValid) {
            img.release();
        }
        img = (cv_bridge::toCvShare(msg, IMAGE_TYPE)->image).clone();
        isValid = true;
    } catch (cv_bridge::Exception& e) {
        std::cout << "ERROR: Could not convert from " << msg->encoding.c_str()
                  << " to " << IMAGE_TYPE.c_str() << "!" << std::endl;
        isValid = false;
    }    
}

int ImagePipeline::getTemplateID(Boxes& boxes) {
    int template_id = -1;
    if(!isValid) 
    {
        std::cout << "ERROR: INVALID IMAGE!" << std::endl;
    }
    else if(img.empty() || img.rows <= 0 || img.cols <= 0) 
    {
        std::cout << "ERROR: VALID IMAGE, BUT STILL A PROBLEM EXISTS!" << std::endl;
        std::cout << "img.empty():" << img.empty() << std::endl;
        std::cout << "img.rows:" << img.rows << std::endl;
        std::cout << "img.cols:" << img.cols << std::endl;
    } 
    else 
    {
        /***YOUR CODE HERE***/
        // Use: boxes.templates
        cv::imshow("view", img);
        cv::waitKey(10);

        int minHessian = 400;
        Ptr<SURF> detector = SURF::create(minHessian);
        FlannBasedMatcher matcher;

        std::vector<KeyPoint> keypoints_object;
        Mat descriptors_object;

        detector->detectAndCompute(img, Mat(), keypoints_object, descriptors_object);

        std::vector<size_t> numMatches(boxes.templates.size(), 0);

        // Go through each box and match
        for (size_t boxInd = 0; boxInd < boxes.templates.size(); ++boxInd)
        {
            auto& boxTemplate = boxes.templates[boxInd];

            Mat descriptors_scene;
            std::vector<KeyPoint> keypoints_scene;
            detector->detectAndCompute(boxTemplate, Mat(), keypoints_scene, descriptors_scene);
            
            // Feature Matching
            std::vector<DMatch> matches;
            matcher.match(descriptors_object, descriptors_scene, matches);

            printf("Number of matches: %d \n", static_cast<int>(matches.size()));

            double max_dist = 0.0, min_dist = 100.0;
            for (size_t i = 0; i < matches.size(); ++i)
            {
                double dist = matches[i].distance;
                min_dist = std::min(min_dist, dist);
                max_dist = std::max(max_dist, dist);
            }

            printf("Min Dist: %lf \n", min_dist);
            printf("Max Dist: %lf \n", max_dist);

            if (min_dist > 0.25)
            {
                break;
            }

            unsigned numGoodMatches = 0;
            for (size_t i = 0; i < matches.size(); ++i)
            {
                if (matches[i].distance < 1.5 * min_dist)
                {
                    ++numGoodMatches;
                }
            }

            printf("Number of good matches: %d \n", numGoodMatches);
            numMatches[boxInd] = numGoodMatches;
        }
        
        size_t maxMatches = 0;
        for (size_t boxInd = 0; boxInd < numMatches.size(); ++boxInd)
        {
            if (maxMatches < numMatches[boxInd])
            {
                template_id = static_cast<int>(boxInd);
                maxMatches = numMatches[boxInd];
            }
        }

        if (maxMatches < 10)
        {
            template_id = -1;
        }
    }  

    if (template_id >= 0)
    {
        cv::imshow("view", boxes.templates[template_id]);
        cv::waitKey(2000);
    }

    return template_id;
}
