#include <imagePipeline.h>

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

void ImagePipeline::populateTemplates(Boxes& boxes)
{
    // Go through each box and match
    for (auto& box : boxes.templates)
    {
        ImageFeatures features;
        detector->detectAndCompute(box, Mat(), features.keypoints, features.descriptors);
        vTemplates.emplace_back(features);
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
        cv::imshow("view", img);
        cv::waitKey(10);
        
        ImageFeatures sceneFeatures;
        detector->detectAndCompute(img, Mat(), sceneFeatures.keypoints, sceneFeatures.descriptors);

        std::vector<size_t> numMatches(vTemplates.size(), 0);

        // Go through each box and match
        for (size_t boxInd = 0; boxInd < vTemplates.size(); ++boxInd)
        {
            auto& templateFeatures = vTemplates[boxInd];
            
            // Feature Matching
            std::vector<DMatch> matches;
            matcher.match(templateFeatures.descriptors, sceneFeatures.descriptors, matches);

            unsigned numGoodMatches = 0;
            for (size_t i = 0; i < matches.size(); ++i)
            {
                if (matches[i].distance < 0.2)
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

        if (maxMatches < 30)
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
