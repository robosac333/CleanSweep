// turtlebot.cpp
#include "turtlebot/turtlebot.hpp"

TurtleBot::TurtleBot() {
}

TurtleBot::TurtleBot(float fSpeed, float rSpeed) {
}

TurtleBot::~TurtleBot() {
}

float TurtleBot::setForwardSpeed(float speed) {
    return 0.0;
}

float TurtleBot::setRotationSpeed(float speed) {
    return 0.0;
}

void TurtleBot::updatePosition(ObstacleAvoidance& obstacleAvoidance) {
}

bool TurtleBot::resetPosition() {
    return false;
}

bool TurtleBot::verifySpeedChange() {
    return false;
}