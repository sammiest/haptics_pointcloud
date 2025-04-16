
#pragma once

#include <HD/hd.h>   // for HHD, HD_INVALID_HANDLE, etc.
#include <vector>
#include <string>
#include <mutex>
#include <Eigen/Dense>

extern "C" {
    typedef unsigned int HDCallbackCode;
    HDCallbackCode __stdcall servoCallback(void* pUserData);
}

/**
 * @brief Minimal 6D Haptic interface named "Touch".
 *
 * Renamed from "HapticInterface" to "Touch" to avoid naming conflicts.
 */

Eigen::Vector3d getPointOnCurve(double t, double mag);

//
double squaredDistance(double t, double mag, const Eigen::Vector3d& ee);

class Touch
{
public:
    Touch();
    ~Touch();

    /**
     * @brief initDevice: Initialize the haptic device.
     * @param configName Optional name/config for your device.
     */
    void initDevice(const std::string &configName = "");

    /**
     * @brief getTransform: Return a 4x4 transform (row-major).
     */
    std::vector<double> getTransform();

    int getButtons();

    /**
     * @brief closeDevice: Close the device and stop the scheduler.
     */
    void closeDevice();

    bool isInitialized() const;

    void setMode(int m);

private:
    bool m_inited;

    /**
     * @brief m_deviceHandle: The HDAPI device handle (HHD).
     *        Use HD_INVALID_HANDLE to indicate an invalid state.
     */
    HHD m_deviceHandle;

    mutable std::mutex _mutex;

    unsigned long _servoCallbackHandle;

    // Store the latest 4x4 transform (row-major).
    double _lastTransform[16];

    int    _lastButtons;

    double _viscosity;

    int    _mode;

    // Mutex to protect _lastTransform.
    // std::mutex _mutex;

    friend HDCallbackCode __stdcall servoCallback(void* pUserData);
};
