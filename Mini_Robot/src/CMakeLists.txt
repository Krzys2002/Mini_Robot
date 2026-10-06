add_library(micro_robot_common INTERFACE)
target_include_directories(micro_robot_common INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(micro_robot_common INTERFACE cxx_std_20)
target_sources(micro_robot_common INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/common/Types.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/RobotId.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/Pose2D.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/Units.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/common/Time.hpp"
)

add_library(micro_robot_camera_api INTERFACE)
target_link_libraries(micro_robot_camera_api INTERFACE micro_robot_common)
target_sources(micro_robot_camera_api INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/camera/CameraTypes.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/camera/ICameraSource.hpp"
)

add_library(micro_robot_transport_api INTERFACE)
target_link_libraries(micro_robot_transport_api INTERFACE micro_robot_common)
target_sources(micro_robot_transport_api INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/transport/TransportTypes.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/transport/IRobotTransport.hpp"
)

add_library(micro_robot_simulation STATIC
    simulation/FakeCameraSource.hpp
    simulation/FakeCameraSource.cpp
    simulation/SimulationTransport.hpp
    simulation/SimulationTransport.cpp
)
target_link_libraries(micro_robot_simulation PUBLIC
    micro_robot_camera_api micro_robot_transport_api
)
micro_robot_target_defaults(micro_robot_simulation)

add_library(micro_robot_ui STATIC
    ui/MainWindow.cpp
    ui/MainWindow.hpp
)
set_target_properties(micro_robot_ui PROPERTIES AUTOMOC ON)
target_include_directories(micro_robot_ui PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(micro_robot_ui PUBLIC Qt6::Widgets)
micro_robot_target_defaults(micro_robot_ui)

add_executable(micro_robot_desktop WIN32 MACOSX_BUNDLE app/main.cpp)
target_link_libraries(micro_robot_desktop PRIVATE micro_robot_ui)
set_target_properties(micro_robot_desktop PROPERTIES
    MACOSX_BUNDLE_BUNDLE_NAME "Micro Robot"
    MACOSX_BUNDLE_GUI_IDENTIFIER "org.microrobot.desktop"
    MACOSX_BUNDLE_BUNDLE_VERSION "${PROJECT_VERSION}"
    MACOSX_BUNDLE_SHORT_VERSION_STRING "${PROJECT_VERSION}"
)
micro_robot_target_defaults(micro_robot_desktop)
