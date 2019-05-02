execute_process(COMMAND "/home/cras/python/aro/hw3/build/exploration/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/cras/python/aro/hw3/build/exploration/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
