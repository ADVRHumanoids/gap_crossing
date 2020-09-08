cmake_minimum_required(VERSION 2.8)
PROJECT(iit-cogimon-ros-pkg)

add_custom_target(  acm
                    COMMAND ${CMAKE_SOURCE_DIR}/cogimon_urdf/script/create_acm.sh cogimon)

add_custom_target(	remove_old_cogimon_urdf_gazebo ALL
    				COMMAND ${CMAKE_COMMAND} -E remove_directory 
        			"~/.gazebo/models/cogimon_urdf/")

add_custom_target(  remove_old_cogimon_gazebo ALL
                    COMMAND ${CMAKE_COMMAND} -E remove_directory 
                    "~/.gazebo/models/cogimon/")

add_custom_target(  compile_urdf_sdf ALL
                    COMMAND ${CMAKE_SOURCE_DIR}/cogimon_urdf/script/create_urdf_srdf_sdf.sh cogimon)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    add_custom_target(	copy_cogimon_gazebo ALL
        				COMMAND ${CMAKE_COMMAND} -E copy_directory 
            			"${PROJECT_SOURCE_DIR}/cogimon_gazebo/sdf/" 
            			"~/.gazebo/models/cogimon/"
            			DEPENDS remove_old_cogimon_gazebo)
endif()
