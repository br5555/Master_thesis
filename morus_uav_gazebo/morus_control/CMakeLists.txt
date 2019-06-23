cmake_minimum_required(VERSION 2.8.3)
project(morus_control)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    rosgraph_msgs
    genmsg
    control_msgs
    sensor_msgs
    morus_msgs
    dynamic_reconfigure
    custom_msgs
    tf
    nav_msgs
    geometry_msgs
)


set (CMAKE_CXX_STANDARD 11)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


set(CMAKE_CONFIGURATION_TYPES "Release" CACHE STRING "" FORCE)
set(Boost_USE_STATIC_LIBS ON) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 

find_package(cmake_modules REQUIRED)

find_package( Boost REQUIRED COMPONENTS regex thread program_options system  )
find_package( Eigen3 REQUIRED   )
#find_package(Ceres REQUIRED)



#include_directories(${CERES_INCLUDE_DIRS})

add_definitions(${EIGEN_DEFINITIONS})

# dynamic reconfigurer
#generate_dynamic_reconfigure_options(config/LinearMPC.cfg)
#include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)

catkin_package(
    INCLUDE_DIRS 
        include 
        lib 
        src 
        config
        
    LIBRARIES morus_control
    CATKIN_DEPENDS
        roscpp
    DEPENDS
        cmake_modules
        Eigen
        #Ceres 
)




include_directories(
 include
  ${catkin_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${Boost_INCLUDE_DIR}
${BOOST_LOCKFREE_DIR}
lib)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
# LIBRARIES #
#############


 add_library(Adam_lib STATIC
   src/MPC_solvers/Adam_MPC.cpp src/attitude_mpc_ctl_adam.cpp src/attitude_teleop_joy.cpp
 )

 add_library(AdaGrad_lib STATIC
   src/MPC_solvers/AdaGrad.cpp 
 )

 add_library(RMS_lib STATIC
   src/MPC_solvers/RMS_Prop_MPC.cpp 
 )
 
  add_library(RMSNester_lib STATIC
   src/MPC_solvers/RMS_Prop_with_Nester_momentum_MPC.cpp 
 )

add_library(PID_lib STATIC
   src/PID.cpp 
 )

target_link_libraries(Adam_lib
 ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} 
)

target_link_libraries(AdaGrad_lib
 ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} 
)


target_link_libraries(RMS_lib
 ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} 
)

target_link_libraries(RMSNester_lib
 ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} 
)

target_link_libraries(PID_lib
 ${catkin_LIBRARIES} 
)

add_executable(adam_node 
        src/nodes/attitude_mpc_ctl_node_adam.cpp  
        )
        
add_executable(lol_ajmo 
        src/probni.cpp  
        )

target_link_libraries(lol_ajmo
        AdaGrad_lib )

target_link_libraries(adam_node
        ${catkin_LIBRARIES} 
        ${EIGEN_LIBRARIES}
        ${Boost_LIBRARIES}
        Adam_lib
        AdaGrad_lib
        RMS_lib
        RMSNester_lib
        PID_lib )

if(FALSE)
add_library(mpc_mm_ceres
        src/mpc_mm_ceres.cpp
        include/morus_control/mpc_mm_ceres.h

)

target_link_libraries(mpc_mm_ceres
 ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${EIGEN_LIBRARIES}
)


# convex solver - CVXGEN
add_library(cvxgen_solver_moving_masses
        lib/ldl.c
        lib/solver.c
        lib/matrix_support.c
        lib/util.c
        lib/variables_definition.c
)

# general algorithm for executing
add_library(attitude_ctl_mpc
        src/attitude_mpc_ctl.cpp
        src/steady_state_calculation.cpp
        src/KFDisturbanceObserver.cpp
        lib/solver.c
)


add_library(test_steady_lib
        include/morus_control/steady_state_calculation.h
        src/steady_state_calculation.cpp

)
target_link_libraries(test_steady_lib
        ${EIGEN_LIBRARIES}
        ${catkin_LIBRARIES}
)


add_library(attitude_ctl_mpc_ceres
        src/attitude_mpc_ctl_ceres.cpp
        src/steady_state_calculation.cpp
        src/KFDisturbanceObserver.cpp
        #include/morus_control/attitude_ctl_mpc_ceres.h
)


target_link_libraries(attitude_ctl_mpc_ceres
        mpc_mm_ceres
        ${catkin_LIBRARIES}
)


add_dependencies(attitude_ctl_mpc
        cvxgen_solver_moving_masses
)



target_link_libraries(attitude_ctl_mpc
        cvxgen_solver_moving_masses
        ${catkin_LIBRARIES}
)



add_library(attitude_teleop_joy
        src/attitude_teleop_joy.cpp
        include/morus_control/attitude_teleop_joy.h)
        
add_library(example_tf_listener_fncs
            src/example_tf_listener_fncs.cpp
            include/morus_control/example_tf_listener.h
)

add_executable(example_tf_listener
        src/example_tf_listener.cpp
        )
add_executable(velocity_in_base_link
        src/velocity_in_base_link.cpp
        )



target_link_libraries(example_tf_listener
        example_tf_listener_fncs
        ${catkin_LIBRARIES})
        
target_link_libraries(velocity_in_base_link
        example_tf_listener_fncs
        ${catkin_LIBRARIES})

add_executable(attitude_mpc_ctl_node
        src/nodes/attitude_mpc_ctl_node.cpp
        )

add_dependencies(attitude_mpc_ctl_node
        ${PROJECT_NAME}_gencfg)

target_link_libraries(attitude_mpc_ctl_node
        ${catkin_LIBRARIES}
        attitude_ctl_mpc
        cvxgen_solver_moving_masses
        attitude_teleop_joy)

add_executable(attitude_mpc_ctl_node_ceres
        src/nodes/attitude_mpc_ctl_node_ceres.cpp
        )


add_executable(test_steady_state
        src/test_steady_state.cpp
        )
add_executable(test_steady_state_heuristic
        src/test_steady_state_heuristic.cpp
        )        
        
        
add_executable(test_derivacije
        src/test_derivacije.cpp
)        
        

target_link_libraries(test_steady_state
        test_steady_lib
        mpc_mm_ceres
        ${EIGEN_LIBRARIES})
        
target_link_libraries(test_steady_state_heuristic
        test_steady_lib
        mpc_mm_ceres
        ${EIGEN_LIBRARIES})

target_link_libraries(test_derivacije

        ${EIGEN_LIBRARIES})



add_dependencies(attitude_mpc_ctl_node_ceres
        ${PROJECT_NAME}_gencfg)

target_link_libraries(attitude_mpc_ctl_node_ceres
        ${catkin_LIBRARIES}
        attitude_ctl_mpc_ceres
        mpc_mm_ceres
        attitude_teleop_joy)
        
endif()
