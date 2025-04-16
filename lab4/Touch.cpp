#include "Touch.hpp"
#include "haptics.hpp"

// OpenHaptics
#include <HD/hd.h>
#include <HD/hdScheduler.h>
#include <Eigen/Dense>

#include <stdexcept>
#include <iostream>
#include <cstring>   // std::memcpy
#include <mutex>     // std::lock_guard
#include <cmath>     // for using velocity, etc.

#define HDCALLBACK __stdcall

const double PI = 3.141592653589793;

/**
 * @brief servoCallback: ~1kHz servo loop.
 *        - read transform + buttons
 *        - read velocity => apply F = -b * velocity
 */
extern "C" HDCallbackCode HDCALLBACK servoCallback(void* pUserData)
{
    auto* touchPtr = static_cast<Touch*>(pUserData);
    if (!touchPtr) return HD_CALLBACK_DONE;

    hdMakeCurrentDevice(touchPtr->m_deviceHandle);

    hdBeginFrame(hdGetCurrentDevice());

    // Read transform in column-major
    double transformCM[16];
    hdGetDoublev(HD_CURRENT_TRANSFORM, transformCM);

    // Convert to row-major
    double transformRM[16];
    for(int r=0; r<4; r++){
        for(int c=0; c<4; c++){
            transformRM[r*4 + c] = transformCM[c*4 + r];
        }
    }

    // Read buttons
    HDint buttons = 0;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &buttons);

    // Read velocity
    double vel3[3];
    hdGetDoublev(HD_CURRENT_VELOCITY, vel3); // in mm/s

    // --------------------DO NOT EDIT ABOVE---------------------
    // ----------------------------------------------------------
    // ---------------------Start of Lab4------------------------

    int mode = 0; //1: spring, 2: sphere, 3: cube, 4: viscosity, 5: friction, 6: line, 7: curve
    {
        std::lock_guard<std::mutex> lk(touchPtr->_mutex);
        mode = touchPtr->_mode;
    }

    double force[3] = {0.0, 0.0, 0.0};
    auto eigen_force = Eigen::Vector3d::Map(force); // N (3x1)
    auto eigen_transform = Eigen::Matrix4d::Map(transformCM); // mm 4x4
    auto eigen_velocity = Eigen::Vector3d::Map(vel3); // mm/s (3x1)

    auto ee_position = eigen_transform.topRightCorner<3,1>(); // (3x1)
    auto base_position = Eigen::Vector3d::Zero(); // (3x1)

    if (mode == 1) { // 1: spring
        eigen_force = spring(ee_position, eigen_velocity);
    } else if (mode == 2) { // 2: sphere
        eigen_force = sphere(ee_position, eigen_velocity);
    } else if (mode == 3) { // 3: cube
        eigen_force = cube(ee_position, eigen_velocity);
    } else if (mode == 4) { // 4: viscosity
        eigen_force = viscosity(ee_position, eigen_velocity);
    } else if (mode == 5) { // 5: friction
        eigen_force = friction(ee_position, eigen_velocity);
    }else if (mode == 6) { // 6: line
        eigen_force = line(ee_position, eigen_velocity);
    } else if (mode == 7) { // 7: curve
        eigen_force = curve(ee_position, eigen_velocity);
    } else if (mode == 8) { // 8: extra credit
        eigen_force = extra(ee_position, eigen_velocity);
    }
    // ---------------------End of Lab4--------------------------
    // ----------------------------------------------------------
    // --------------------DO NOT EDIT BELOW---------------------

    hdSetDoublev(HD_CURRENT_FORCE, force);

    // Store transform + buttons
    {
        std::lock_guard<std::mutex> lk(touchPtr->_mutex);
        std::memcpy(touchPtr->_lastTransform, transformRM, sizeof(transformRM));
        touchPtr->_lastButtons = static_cast<int>(buttons);
    }

    hdEndFrame(hdGetCurrentDevice());
    return HD_CALLBACK_CONTINUE;
}

Touch::Touch()
    : m_inited(false)
    , m_deviceHandle(HD_INVALID_HANDLE)
    , _servoCallbackHandle(0)
    , _lastButtons(0)
    , _viscosity(0.0)
    , _mode(0)
{
    for(int i=0; i<16; i++){
        _lastTransform[i] = ((i%5)==0) ? 1.0 : 0.0;
    }
}

Touch::~Touch()
{
    if(m_inited) closeDevice();
}

void Touch::initDevice(const std::string &configName)
{
    if(m_inited){
        throw std::runtime_error("[Touch] Already inited!");
    }

    m_deviceHandle = hdInitDevice(configName.empty() ? HD_DEFAULT_DEVICE : configName.c_str());


    // check error
    {
        HDErrorInfo err = hdGetError();
        if(HD_DEVICE_ERROR(err)){
            throw std::runtime_error("[Touch] hdInitDevice failed code=" +
                                      std::to_string(err.errorCode));
        }
    }

    // 2) **Enable force output** so the device can render forces
    hdEnable(HD_FORCE_OUTPUT);

    // (Optional) Force ramping can help reduce sudden jolts
    hdEnable(HD_FORCE_RAMPING);

    hdMakeCurrentDevice(m_deviceHandle);

    // 2) Start scheduler
    hdStartScheduler();
    {
        HDErrorInfo err = hdGetError();
        if(HD_DEVICE_ERROR(err)){
            hdDisableDevice(m_deviceHandle);
            throw std::runtime_error("[Touch] hdStartScheduler failed code=" +
                                     std::to_string(err.errorCode));
        }
    }

    std::cout << "[Touch] Device init ok.\n";

    // 3) schedule servo callback
    _servoCallbackHandle = hdScheduleAsynchronous(
        servoCallback,
        this,
        HD_DEFAULT_SCHEDULER_PRIORITY
    );

    // m_deviceHandle = hHD;
    m_inited = true;
}

void Touch::closeDevice()
{
    if(!m_inited) return;

    if(_servoCallbackHandle){
        hdUnschedule(_servoCallbackHandle);
        _servoCallbackHandle = 0;
    }

    hdStopScheduler();
    hdDisableDevice(m_deviceHandle);

    m_deviceHandle = HD_INVALID_HANDLE;
    m_inited = false;
    std::cout << "[Touch] Device closed.\n";
}

std::vector<double> Touch::getTransform()
{
    if(!m_inited){
        throw std::runtime_error("[Touch] Not inited yet!");
    }
    std::lock_guard<std::mutex> lk(_mutex);

    std::vector<double> out(16);
    for(int i=0; i<16; i++){
        out[i] = _lastTransform[i];
    }
    return out;
}

int Touch::getButtons()
{
    if(!m_inited){
        throw std::runtime_error("[Touch] Not inited yet!");
    }
    std::lock_guard<std::mutex> lk(_mutex);
    return _lastButtons;
}

bool Touch::isInitialized() const
{
    return m_inited;
}

void Touch::setMode(int m)
{
    std::lock_guard<std::mutex> lk(_mutex);
    _mode = m;
}
