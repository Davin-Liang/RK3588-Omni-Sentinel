#ifndef ALARM_GPIO_H
#define ALARM_GPIO_H

#include <mutex>
#include <string>

/**
 * @brief RK3588 alarm GPIO helper based on Linux sysfs GPIO.
 *
 * Default project wiring:
 *   P26-32 / GPIO3_A1 / gpio97 -> STM32 EXTI input
 *   P26-34 / GND               -> STM32 GND
 *
 * activeLow=true means:
 *   inactive/no alarm: physical high level
 *   active/alarm    : physical low level
 */
class AlarmGpio
{
public:
    AlarmGpio() = default;

    bool init(int gpioNum = 97, bool activeLow = true);
    bool setAlarmActive(bool active);
    bool setHigh();
    bool setLow();
    bool isReady() const;

private:
    bool writeFile(const std::string& path, const std::string& value) const;
    bool pathExists(const std::string& path) const;

private:
    mutable std::mutex mutex_;
    int gpioNum_ = -1;
    bool activeLow_ = true;
    bool ready_ = false;
};

#endif // ALARM_GPIO_H
