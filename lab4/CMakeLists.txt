find_package(pybind11 CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

include_directories("C:/OpenHaptics/Developer/3.5.0/include")
link_directories("C:/OpenHaptics/Developer/3.5.0/lib/x64/Release")
include_directories("C:/Users/FRBGuest/Desktop/Lab4/medrob-lab4-main/lab4")


pybind11_add_module(touch
    haptics.cpp haptics.hpp
    Touch.cpp Touch.hpp
    Touch_bindings.cpp
)

set_target_properties(touch PROPERTIES OUTPUT_NAME "touch.cp310-win_amd64.pyd")

target_link_libraries(touch PRIVATE
    hd.lib
    Eigen3::Eigen
)
