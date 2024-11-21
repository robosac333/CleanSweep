#include "cleansweep/object_detection.hpp"

ObjectDetection::ObjectDetection()
    : Node("object_detection_node"),
      processingRate(30.0),
      detectionStatus(false),
      maxThreshold(255),
      minThreshold(0) {
    
    imageSubscriber = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/image_raw", 10,
        std::bind(&ObjectDetection::processInput, this, std::placeholders::_1));
}

ObjectDetection::~ObjectDetection() {
}

void ObjectDetection::processInput(const std::shared_ptr<sensor_msgs::msg::Image> imageData) {
}

bool ObjectDetection::findTarget(cv::Mat image) {
    return false;
}

cv::Mat ObjectDetection::applyFilter(cv::Mat image) {
    return cv::Mat();
}

cv::Rect ObjectDetection::getTargetArea() {
    return targetRegion;
}

void ObjectDetection::setTargetArea(cv::Rect rect) {
    targetRegion = rect;
}

bool ObjectDetection::getDetectionStatus() {
    return detectionStatus;
}

void ObjectDetection::setDetectionStatus(bool status) {
    detectionStatus = status;
}

// int main(int argc, char* argv[]) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<ObjectDetection>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }