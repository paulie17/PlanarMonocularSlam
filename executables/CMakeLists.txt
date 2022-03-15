add_executable(main main.cpp
../src/dataset_utils.cpp
../src/camera.cpp
../src/pms_solver.cpp
)
target_link_libraries(main ${Boost_LIBRARIES})

