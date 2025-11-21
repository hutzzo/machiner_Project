cmake_minimum_required(VERSION 3.5)
project(bodyreader)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
#find_package(actionlib REQUIRED)
#find_package(actionlib_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(bodyreader_msg REQUIRED)
find_package(image_transport REQUIRED)                         # CHANGE
find_package(cv_bridge REQUIRED)  
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
#rosidl_generate_interfaces(${PROJECT_NAME}
# )


link_directories(lib)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include
)

link_directories(
  lib lib/Plugins lib/Plugins/openni2 lib/Plugins/obt/3rdparty/lib
)

# Detect Orbbec Astra SDK libraries (local vendor libs)
set(_ASTRA_LIB_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/Plugins
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/Plugins/openni2
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/Plugins/obt/3rdparty/lib
)

find_library(ASTRA_LIB
  NAMES astra libastra.so
  PATHS ${_ASTRA_LIB_DIRS}
  NO_DEFAULT_PATH
)
find_library(ASTRA_CORE_LIB
  NAMES astra_core libastra_core.so
  PATHS ${_ASTRA_LIB_DIRS}
  NO_DEFAULT_PATH
)
find_library(ASTRA_CORE_API_LIB
  NAMES astra_core_api libastra_core_api.so
  PATHS ${_ASTRA_LIB_DIRS}
  NO_DEFAULT_PATH
)

# External Astra SDK via environment
set(ASTRA_SDK_INCLUDE $ENV{ASTRA_SDK_INCLUDE})
set(ASTRA_SDK_LIB $ENV{ASTRA_SDK_LIB})
if(ASTRA_SDK_INCLUDE)
  include_directories(${ASTRA_SDK_INCLUDE})
endif()
if(ASTRA_SDK_LIB)
  link_directories(${ASTRA_SDK_LIB})
endif()
find_library(ASTRA_LIB_EXT
  NAMES astra libastra.so libastra astra.so
  PATHS ${ASTRA_SDK_LIB} ${ASTRA_SDK_LIB}/lib ${ASTRA_SDK_LIB}/Plugins
  NO_DEFAULT_PATH)
find_library(ASTRA_CORE_LIB_EXT
  NAMES astra_core libastra_core.so astra_core.so libastra_core
  PATHS ${ASTRA_SDK_LIB} ${ASTRA_SDK_LIB}/lib ${ASTRA_SDK_LIB}/Plugins
  NO_DEFAULT_PATH)
find_library(ASTRA_CORE_API_LIB_EXT
  NAMES astra_core_api libastra_core_api.so astra_core_api.so libastra_core_api
  PATHS ${ASTRA_SDK_LIB} ${ASTRA_SDK_LIB}/lib ${ASTRA_SDK_LIB}/Plugins
  NO_DEFAULT_PATH)

include(CheckLibraryExists)
set(_ASTRA_CHECK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib")
check_library_exists(astra astra_initialize "${_ASTRA_CHECK_PATH}" ASTRA_LINK_OK)
check_library_exists(astra_core astra_core_initialize "${_ASTRA_CHECK_PATH}" ASTRA_CORE_LINK_OK)
check_library_exists(astra_core_api astra_core_api_initialize "${_ASTRA_CHECK_PATH}" ASTRA_CORE_API_LINK_OK)

set(HAVE_ASTRA_OK FALSE)
if((ASTRA_LIB AND ASTRA_CORE_LIB AND ASTRA_CORE_API_LIB AND ASTRA_LINK_OK AND ASTRA_CORE_LINK_OK AND ASTRA_CORE_API_LINK_OK)
   OR (ASTRA_LIB_EXT AND ASTRA_CORE_LIB_EXT AND ASTRA_CORE_API_LIB_EXT))
  set(HAVE_ASTRA_OK TRUE)
else()
  message(WARNING "Astra SDK libraries missing or incompatible; bodyreader nodes will be skipped.")
endif()
if(NOT ((ASTRA_LIB AND ASTRA_CORE_LIB AND ASTRA_CORE_API_LIB) OR (ASTRA_LIB_EXT AND ASTRA_CORE_LIB_EXT AND ASTRA_CORE_API_LIB_EXT)))
  message(WARNING "Astra SDK libraries not found or incompatible; skipping bodyreader binaries.")
endif()

#add_executable(wheeltec_robot_node src/wheeltec_robot.cpp src/Quaternion_Solution.cpp)
#ament_target_dependencies(wheeltec_robot_node tf2_ros tf2 tf2_geometry_msgs rclcpp std_msgs nav_msgs std_srvs sensor_msgs ackermann_msgs serial wheeltec_robot_msg)

if(HAVE_ASTRA_OK)
  add_executable(main src/main.cpp)
  ament_target_dependencies(main rclcpp bodyreader_msg sensor_msgs)
  if(ASTRA_LIB_EXT)
    target_link_libraries(main ${ASTRA_LIB_EXT} ${ASTRA_CORE_LIB_EXT} ${ASTRA_CORE_API_LIB_EXT})
  else()
    target_link_libraries(main ${ASTRA_LIB} ${ASTRA_CORE_LIB} ${ASTRA_CORE_API_LIB})
  endif()

  add_executable(bodydata_process src/bodydata_process.cpp)
  ament_target_dependencies(bodydata_process rclcpp bodyreader_msg sensor_msgs std_msgs geometry_msgs)
  if(ASTRA_LIB_EXT)
    target_link_libraries(bodydata_process ${ASTRA_LIB_EXT} ${ASTRA_CORE_LIB_EXT} ${ASTRA_CORE_API_LIB_EXT})
  else()
    target_link_libraries(bodydata_process ${ASTRA_LIB} ${ASTRA_CORE_LIB} ${ASTRA_CORE_API_LIB})
  endif()

  add_executable(follower src/follower.cpp)
  ament_target_dependencies(follower rclcpp bodyreader_msg sensor_msgs std_msgs geometry_msgs)
  if(ASTRA_LIB_EXT)
    target_link_libraries(follower ${ASTRA_LIB_EXT} ${ASTRA_CORE_LIB_EXT} ${ASTRA_CORE_API_LIB_EXT})
  else()
    target_link_libraries(follower ${ASTRA_LIB} ${ASTRA_CORE_LIB} ${ASTRA_CORE_API_LIB})
  endif()

  add_executable(interaction src/interaction.cpp)
  ament_target_dependencies(interaction rclcpp bodyreader_msg geometry_msgs)
  if(ASTRA_LIB_EXT)
    target_link_libraries(interaction ${ASTRA_LIB_EXT} ${ASTRA_CORE_LIB_EXT} ${ASTRA_CORE_API_LIB_EXT})
  else()
    target_link_libraries(interaction ${ASTRA_LIB} ${ASTRA_CORE_LIB} ${ASTRA_CORE_API_LIB})
  endif()

  add_executable(feedback src/feedback.cpp)
  ament_target_dependencies(feedback rclcpp bodyreader_msg geometry_msgs)
  if(ASTRA_LIB_EXT)
    target_link_libraries(feedback ${ASTRA_LIB_EXT} ${ASTRA_CORE_LIB_EXT} ${ASTRA_CORE_API_LIB_EXT})
  else()
    target_link_libraries(feedback ${ASTRA_LIB} ${ASTRA_CORE_LIB} ${ASTRA_CORE_API_LIB})
  endif()

  add_executable(image_trans src/image_trans.cpp)
  ament_target_dependencies(image_trans rclcpp bodyreader_msg sensor_msgs image_transport)
  if(ASTRA_LIB_EXT)
    target_link_libraries(image_trans ${ASTRA_LIB_EXT} ${ASTRA_CORE_LIB_EXT} ${ASTRA_CORE_API_LIB_EXT})
  else()
    target_link_libraries(image_trans ${ASTRA_LIB} ${ASTRA_CORE_LIB} ${ASTRA_CORE_API_LIB})
  endif()
endif()


install(PROGRAMS scripts/display.py scripts/compressed.py DESTINATION lib/${PROJECT_NAME})

if(HAVE_ASTRA_OK)
  install(TARGETS
    main bodydata_process follower interaction feedback image_trans
    DESTINATION lib/${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endif()
install(
  DIRECTORY launch audio
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
