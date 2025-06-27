cmake_minimum_required(VERSION 3.8)
project(gazebo_vacuum_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
if("$ENV{GZ_VERSION}" STREQUAL "harmonic" OR NOT DEFINED "$ENV{GZ_VERSION}")
    find_package(gz-plugin2 REQUIRED COMPONENTS register)
  find_package(gz-cmake3 REQUIRED)
  find_package(gz-sim8 REQUIRED)
  find_package(gz-math7 REQUIRED)
  set(GZ_PLUGIN_VER  ${gz-plugin2_VERSION_MAJOR})
  set(GZ_COMMON_VER  ${gz-common5_VERSION_MAJOR})
  set(GZ_MATH_VER  ${gz-math7_VERSION_MAJOR})
  set(GZ_SIM_VER  ${gz-sim8_VERSION_MAJOR})
endif()



add_library(VacuumGripperPlugin
            SHARED
            src/VacuumPlugin.cc)
include_directories(VacuumGripperPlugin PRIVATE include)
target_link_libraries(VacuumGripperPlugin
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-math${GZ_MATH_VER}::eigen3
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

install(
  TARGETS VacuumGripperPlugin DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()

