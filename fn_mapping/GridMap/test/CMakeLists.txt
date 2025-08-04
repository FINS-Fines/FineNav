ament_add_gtest(${PROJECT_NAME}_test
        unit/grid_map_math_test.cpp
        unit/grid_map_test.cpp
)

target_link_libraries(${PROJECT_NAME}_test
        fn_grid_map
)