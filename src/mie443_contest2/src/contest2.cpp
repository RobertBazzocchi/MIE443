#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <math.h>
#include <cmath>

#include <fstream>

struct pose
{
    float x;
    float y;
    float phi;
};

float euclideanDistance(pose currentCoord, pose nextCoord)
{
    float dist = sqrt( pow(currentCoord.x-nextCoord.x,2)+ pow(currentCoord.y-nextCoord.y,2));
    float angleDiff = abs(currentCoord.phi - fmod(nextCoord.phi + 3.14, 6.28));
    float totalCost = dist + 0.7*angleDiff;
    return totalCost;
}

std::vector<pose> getBinOrder(RobotPose startingRobotPose, Boxes boxes)
{
    std::vector<pose> orderedCoords;
    std::vector<pose> remainingCoords;

    for (auto& coord : boxes.coords)
    {
        remainingCoords.push_back({coord[0], coord[1], coord[2]});
    }
    
    orderedCoords.push_back({startingRobotPose.x,startingRobotPose.y,startingRobotPose.phi});   
    while(orderedCoords.size() < boxes.coords.size() + 1) 
    {
        float minDist = 1000; 
        int targetIndex = 0;
        pose currentCoord = orderedCoords.back();
        for(int i = 0; i < remainingCoords.size(); ++i) 
        {
            float dist = euclideanDistance(currentCoord, remainingCoords[i]);
            if (dist < minDist)
            {
                minDist = dist;
                targetIndex = i;
            }            
        }

        orderedCoords.push_back(remainingCoords[targetIndex]);
        remainingCoords.erase(remainingCoords.begin() + targetIndex);
    }
    
    orderedCoords.push_back({startingRobotPose.x,startingRobotPose.y,startingRobotPose.phi});
    orderedCoords.erase(orderedCoords.begin());
    return orderedCoords;
}

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;

    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);

    // Initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) 
    {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }

    for(int i = 0; i < boxes.coords.size(); ++i) 
    {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
                  << boxes.coords[i][2] << std::endl;
    }

    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);

    // Initalize navigator
    Navigation navigator;

    // Initalize which box is currently being navigated to
    unsigned boxInd = 0;  

    std::ofstream BoxIDs("BoxIDs.txt");

    // Make sure initalization completes
    for (auto i = 0; i < 30; ++i)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

    // Get Localization initalized
    for (auto i = 0; i < 5; ++i)
    {
        ros::spinOnce();
        if (!navigator.moveToGoal(robotPose.x + 0.01, robotPose.y, robotPose.phi))
        {
            return -1;
        }
    }

    // Get box order to follow and current box
    auto boxOrder = getBinOrder(robotPose, boxes);
    auto currBox = boxOrder.begin();

    // Execute strategy.
    while(ros::ok()) 
    {
        ros::spinOnce();
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi

        float phi = currBox->phi;
        float x = currBox->x + 0.7*std::cos(phi);
        float y = currBox->y + 0.7*std::sin(phi);
        phi = fmod(phi + 3.14, 6.28);

        if (!navigator.moveToGoal(x, y, phi))
        {
            return -1;
        }

        // Check stop condition
        if (currBox == boxOrder.end())
        {
            std::cout << "Found all boxes" << std::endl;
            return 0;
        }
        else
        {
             // Get template ID and write to file
            auto templateID = imagePipeline.getTemplateID(boxes);
            BoxIDs << templateID << std::endl;

            ++currBox;
            ros::Duration(0.01).sleep();
        }
        
    }
    return 0;
}
