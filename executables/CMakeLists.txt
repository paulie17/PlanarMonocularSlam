add_executable(main main.cpp
../src/dataset_utils.cpp
../src/camera.cpp
../src/pms_solver.cpp
)
target_link_libraries(main ${Boost_LIBRARIES})

#add_executable(g2o_test g2o_test.cpp)
#target_link_libraries(g2o_test 
#g2o_stuff 
#g2o_core
#g2o_solver_eigen
#)
