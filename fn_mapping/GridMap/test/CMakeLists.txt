ament_add_gtest(${PROJECT_NAME}_unit_test
        unit/grid_map_math_test.cpp
        unit/grid_map_test.cpp
        unit/get_difference_set_test.cpp
)
target_include_directories(${PROJECT_NAME}_unit_test PRIVATE utils)
target_link_libraries(${PROJECT_NAME}_unit_test fn_grid_map)


ament_add_gtest(${PROJECT_NAME}_integration_test
        integration/grid_map_test.cpp
)
target_include_directories(${PROJECT_NAME}_integration_test PRIVATE utils)
target_link_libraries(${PROJECT_NAME}_integration_test fn_grid_map)
