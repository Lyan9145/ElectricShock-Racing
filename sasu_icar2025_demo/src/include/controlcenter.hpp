#pragma once

#include <string>
#include <vector>
#include <cmath>
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "common.hpp"
#include "detection.hpp"
#include "json.hpp"
#include "motion.hpp"
#include "thread.hpp"
#include "uart.hpp"
#include "tracking.hpp"

class ControlCenter {
public:
    ControlCenter();
    ~ControlCenter();

    void initialize();
    void start();
    void stop();
    void reset();

private:
    void loadConfiguration(const std::string& configFile);
    void processDetection();
    void handleMotion();
    void manageThreads();
    void communicateWithUart();

    // Add other private methods and members as needed
};

