#pragma once

#include <string>
#include <vector>
#include "common.hpp"
#include "detection.hpp"
#include "json.hpp"
#include "motion.hpp"
#include "thread.hpp"
#include "uart.hpp"

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

