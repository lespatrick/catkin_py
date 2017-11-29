execute_process(COMMAND "/home/leszek/catkin_py/catkin_robo_ws/build/motor_hat_receiver/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/leszek/catkin_py/catkin_robo_ws/build/motor_hat_receiver/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
