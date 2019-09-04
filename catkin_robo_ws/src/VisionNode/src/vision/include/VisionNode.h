//
// Created by leszek on 23.06.19.
//

#include "ros/ros.h"

#include "QrReader.h"
#include "DistanceCalculator.h"

#include "sensor_msgs/Image.h"
#include <geometry_msgs/PoseStamped.h>

#include "zbar.h"
#include "json.hpp"
#include "Location.h"

#ifndef SRC_VISIONNODE_H
#define SRC_VISIONNODE_H


class VisionNode {
public:
    VisionNode() : _distCalc(ModelHeight, ModelDistance) {
        _imageSub = n.subscribe("/raspicam_node/image_raw", 1, &VisionNode::processImage, this);
        _locationPublisher = n.advertise<vision_msgs::Location>("/newLocation", 1);
        ROS_INFO("VisionNode initialized");
    };
private:
    double const ModelHeight = 121.0;
    double const ModelDistance = 0.6;

    ros::NodeHandle n;

    QrReader _reader;
    DistanceCalculator _distCalc;

    ros::Subscriber _imageSub;
    ros::Publisher _locationPublisher;
    ros::Publisher _poiPublisher;

    void processImage(const sensor_msgs::Image &image);
    void newLocationRecognised(const nlohmann::json &message, int targetWidth, int targetHeight);
    void newPOIRecognised(const nlohmann::json &message);
};


#endif //SRC_VISIONNODE_H
