execute_process(COMMAND "/home/warp/catkin_ws/src/rover_base/rover_gps_widget/build/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/warp/catkin_ws/src/rover_base/rover_gps_widget/build/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
