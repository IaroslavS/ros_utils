set(EXECUTABLE_TARGETS "")

add_executable(listen_and_save listen_and_save.cc)
list(APPEND EXECUTABLE_TARGETS listen_and_save)

foreach(EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)
    add_dependencies(${EXECUTABLE_TARGET} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

endforeach()
