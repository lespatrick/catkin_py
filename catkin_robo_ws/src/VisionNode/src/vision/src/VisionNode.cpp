//
// Created by leszek on 23.06.19.
//

#include "VisionNode.h"
#include "cv_bridge/cv_bridge.h"
#include "Location.h"

void VisionNode::processImage(const sensor_msgs::Image &image) {
    ROS_INFO("Image received");
    cv_bridge::CvImageConstPtr cv_image;
    cv_image = cv_bridge::toCvCopy(image, "mono8");

    zbar::Image zbar_image(cv_image->image.cols, cv_image->image.rows, "Y800", cv_image->image.data,
                           cv_image->image.cols * cv_image->image.rows);

    _reader.processImage(zbar_image, [this](std::string text, int targetWidth, int targetHeight) {
        using json = nlohmann::json;
        json message = json::parse(text);
        ROS_INFO(text.c_str());

        newLocationRecognised(message, targetWidth, targetHeight);
    });
}

void VisionNode::newLocationRecognised(const nlohmann::json &message, int targetWidth, int targetHeight) {
    std::string _mapName = message["name"];
    double _xPos = message["xPos"];
    double _yPos = message["yPos"];
    double _xNormal = message["xNormal"];
    double _yNormal = message["yNormal"];

    double distance = _distCalc.calculateDistance(targetHeight);

    double _normalisedXPos = _xPos + _xNormal * distance;
    double _normalisedYPos = _yPos + _yNormal * distance;

    vision_msgs::Location newLocation;
    newLocation.map_name = _mapName;
    newLocation.x_pos = _normalisedXPos;
    newLocation.y_pos = _normalisedYPos;

    _locationPublisher.publish(newLocation);
}

void VisionNode::newPOIRecognised(const nlohmann::json &message) {

}