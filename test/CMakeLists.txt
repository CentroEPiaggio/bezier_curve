## Declare a C++ executable
add_executable(${PROJECT_NAME}_test bezier_curve_test.cpp)
add_dependencies(${PROJECT_NAME}_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test
    ${PROJECT_NAME}_lib
    ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(bc_test
    bc_test.cpp
  )
  target_link_libraries(bc_test
    ${PROJECT_NAME}_lib
    ${catkin_LIBRARIES}
    ${GTEST_LIBRARIES}
    # ${Boost_LIBRARIES}
  )
endif()