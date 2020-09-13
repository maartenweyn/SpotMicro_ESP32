# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)


list(APPEND EXTRA_COMPONENT_DIRS "components/pca9685")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(pca9685_pwm_test)

