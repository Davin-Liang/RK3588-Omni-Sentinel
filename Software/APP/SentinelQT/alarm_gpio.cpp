#include "alarm_gpio.h"

#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>

bool AlarmGpio::pathExists(const std::string& path) const
{
    return access(path.c_str(), F_OK) == 0;
}

bool AlarmGpio::writeFile(const std::string& path, const std::string& value) const
{
    std::ofstream ofs(path);
    if (!ofs.is_open()) {
        std::cerr << "[AlarmGPIO] failed to open: " << path << std::endl;
        return false;
    }
    ofs << value;
    if (!ofs.good()) {
        std::cerr << "[AlarmGPIO] failed to write " << value
                  << " to " << path << std::endl;
        return false;
    }
    return true;
}

bool AlarmGpio::init(int gpioNum, bool activeLow)
{
    std::lock_guard<std::mutex> lock(mutex_);

    gpioNum_ = gpioNum;
    activeLow_ = activeLow;
    ready_ = false;

    const std::string gpioPath = "/sys/class/gpio/gpio" + std::to_string(gpioNum_);

    if (!pathExists(gpioPath)) {
        // If export returns "Device or resource busy" but gpioPath appears later,
        // this still counts as success.
        writeFile("/sys/class/gpio/export", std::to_string(gpioNum_));
        usleep(100 * 1000);
    }

    if (!pathExists(gpioPath)) {
        std::cerr << "[AlarmGPIO] gpio" << gpioNum_
                  << " is not available under /sys/class/gpio" << std::endl;
        return false;
    }

    // Keep physical logic normal: value=1 means high level, value=0 means low level.
    // Active-low behavior is handled by setAlarmActive().
    writeFile(gpioPath + "/active_low", "0");

    if (!writeFile(gpioPath + "/direction", "out")) {
        return false;
    }

    ready_ = true;

    // Default state: no alarm. For active-low wiring this is physical high level.
    return writeFile(gpioPath + "/value", activeLow_ ? "1" : "0");
}

bool AlarmGpio::setAlarmActive(bool active)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!ready_ || gpioNum_ < 0) {
        return false;
    }

    const std::string valuePath =
        "/sys/class/gpio/gpio" + std::to_string(gpioNum_) + "/value";

    const char* value = nullptr;
    if (activeLow_) {
        value = active ? "0" : "1";   // alarm -> low, no alarm -> high
    } else {
        value = active ? "1" : "0";
    }

    return writeFile(valuePath, value);
}

bool AlarmGpio::setHigh()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ready_ || gpioNum_ < 0) {
        return false;
    }
    const std::string valuePath =
        "/sys/class/gpio/gpio" + std::to_string(gpioNum_) + "/value";
    return writeFile(valuePath, "1");
}

bool AlarmGpio::setLow()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ready_ || gpioNum_ < 0) {
        return false;
    }
    const std::string valuePath =
        "/sys/class/gpio/gpio" + std::to_string(gpioNum_) + "/value";
    return writeFile(valuePath, "0");
}

bool AlarmGpio::isReady() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return ready_;
}
