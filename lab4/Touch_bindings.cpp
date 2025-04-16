
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "Touch.hpp"

namespace py = pybind11;

/**
 * @brief This pybind11 module is named "touch" instead of "interface".
 *        We'll define a Python class "Touch" wrapping our C++ "Touch".
 */
PYBIND11_MODULE(touch, m)
{
    m.doc() = R"pbdoc(
        Python bindings for the "Touch" class (OpenHaptics HDAPI).
        Example usage:
            from igmr-robotics-toolkit.io.3DSystem_Haptic.touch import Touch
            dev = Touch()
            dev.init_device()
            matrix = dev.getTransform()
    )pbdoc";

    py::class_<Touch>(m, "Touch")
        .def(py::init<>())

        .def("init_device", &Touch::initDevice,
             py::arg("configName") = "")
        .def("close_device", &Touch::closeDevice)
        .def("get_transform", &Touch::getTransform)
        .def("get_buttons", &Touch::getButtons, R"pbdoc(Return integer bitmask of button states)pbdoc")
        
        .def("is_initialized", &Touch::isInitialized,
             "Check if device is properly initialized")

        .def("set_mode",
            &Touch::setMode,
            py::arg("m"),
            R"pbdoc(Set the haptic mode used for this setup.)pbdoc");
}