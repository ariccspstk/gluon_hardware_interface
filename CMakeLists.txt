cmake_minimum_required(VERSION 3.8)
project(gluon_hardware_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)

# ---------------------------------------------------------------------------
# 1. Bring in innfos-cpp-sdk, BEFORE anything ros2_control.
#
# innfos-cpp-sdk ships a PREBUILT shared library, not sources to compile:
#   external/innfos-cpp-sdk/sdk/lib/linux_x86_64/libActuatorController.so
#   external/innfos-cpp-sdk/sdk/include/            <- ActuatorController.h etc.
#
# Traced ActuatorController.h's full transitive #include chain by hand
# (actuatorcontroller_global.h, actuatordefine.h, CSignal.hpp, stdlib
# headers): none of it touches asio, the serial library, or pthreads. Those
# only appear in the SDK's private implementation headers (itimer.h,
# EthernetCommunicateUnit.h, etc.), which are already compiled into the
# prebuilt .so and never included by anything a consumer touches. So unlike
# upstream's own example/CMakeLists.txt (which builds those internal
# headers directly and does need ASIO_STANDALONE/ASIO_HAS_STD_CHRONO/
# pthread), none of that is load-bearing here - confirmed by a clean build
# without them.
#
# Since it's a binary blob, we declare it as an IMPORTED target rather than
# add_library(... SHARED <sources>) - there is nothing for CMake to compile
# here, only to link against and ship alongside our own plugin.
# ---------------------------------------------------------------------------
set(INNFOS_SDK_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/external/innfos-cpp-sdk/sdk)
set(INNFOS_SDK_INCLUDE_DIR ${INNFOS_SDK_ROOT}/include)
set(INNFOS_SDK_LIB ${INNFOS_SDK_ROOT}/lib/linux_x86_64/libActuatorController.so)

if(NOT EXISTS ${INNFOS_SDK_LIB})
  message(FATAL_ERROR
    "innfos-cpp-sdk prebuilt library not found at ${INNFOS_SDK_LIB}.\n"
    "Add it first, e.g.:\n"
    "  git submodule add https://github.com/mintasca/innfos-cpp-sdk.git external/innfos-cpp-sdk\n"
    "  git submodule update --init --recursive"
  )
endif()

if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  message(WARNING
    "innfos-cpp-sdk only ships a prebuilt linux_x86_64 .so. "
    "Building on ${CMAKE_SYSTEM_PROCESSOR} (e.g. an ARM SBC) will configure "
    "fine but fail at link/runtime unless you get an arm64 build of "
    "libActuatorController.so from INNFOS."
  )
endif()

add_library(ActuatorController SHARED IMPORTED GLOBAL)
set_target_properties(ActuatorController PROPERTIES
  IMPORTED_LOCATION ${INNFOS_SDK_LIB}
  INTERFACE_INCLUDE_DIRECTORIES ${INNFOS_SDK_INCLUDE_DIR}
)

# ---------------------------------------------------------------------------
# 2. The actual ros2_control SystemInterface plugin, linked against
#    ActuatorController above.
# ---------------------------------------------------------------------------
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(gluon_hardware_interface SHARED
  src/gluon_hardware_interface.cpp
)
target_compile_features(gluon_hardware_interface PUBLIC cxx_std_17)
target_include_directories(gluon_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(gluon_hardware_interface PUBLIC ActuatorController)
ament_target_dependencies(gluon_hardware_interface PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface gluon_hardware_interface.xml)

# ---------------------------------------------------------------------------
# Install
#
# ActuatorController is IMPORTED (a binary we didn't build), so it can't go
# through install(TARGETS ...) / EXPORT like a normal target - CMake has no
# build rule to "install" an import, only the underlying file. We copy the
# .so directly. gluon_hardware_interface, by contrast, IS a target CMake
# built, so it goes through the normal install(TARGETS ... EXPORT ...) path.
# ---------------------------------------------------------------------------
install(
  FILES ${INNFOS_SDK_LIB}
  DESTINATION lib
)

install(
  DIRECTORY ${INNFOS_SDK_INCLUDE_DIR}/
  DESTINATION include/${PROJECT_NAME}/ActuatorController
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS gluon_hardware_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
# ActuatorController itself is IMPORTED and can't be part of an EXPORT set,
# so it's still exported the old way, by name, alongside the target export.
ament_export_libraries(ActuatorController)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()