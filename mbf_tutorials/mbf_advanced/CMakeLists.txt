cmake_minimum_required(VERSION 2.8.3)
project(mbf_advanced)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(behaviortree_cpp_v3 REQUIRED)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  actionlib
  mbf_msgs
  std_msgs
  odombase
)

catkin_package(
  DEPENDS move_base_flex
)

catkin_install_python(
  PROGRAMS scripts/circle_smach.py scripts/pytrees.py scripts/mbf_client.py scripts/battery_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(mbf_client_node src/mbf_client_node.cpp)
target_include_directories(mbf_client_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(mbf_client_node ${catkin_LIBRARIES})
target_compile_definitions(mbf_client_node
        PRIVATE
        POSE_PATH="${PROJECT_SOURCE_DIR}/param/poses.txt"
)

add_executable(mbf_behavior_tree src/mbf_behavior_tree.cpp)
target_include_directories(mbf_behavior_tree PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(mbf_behavior_tree ${behaviortree_cpp_v3_LIBRARIES} ${catkin_LIBRARIES})
target_compile_definitions(mbf_behavior_tree
        PRIVATE
        BT_XML_PATH="${PROJECT_SOURCE_DIR}/param/mbf_behavior_tree.xml"
        POSE_PATH="${PROJECT_SOURCE_DIR}/param/poses.txt"
        	AW="Yi"
)

add_executable(mbf_bt_v1 src/mbf_bt_v1.cpp)
target_include_directories(mbf_bt_v1 PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(mbf_bt_v1 ${behaviortree_cpp_v3_LIBRARIES} ${catkin_LIBRARIES})
target_compile_definitions(mbf_bt_v1
        PRIVATE

        BT_XML_PATH2="${PROJECT_SOURCE_DIR}/param/mbf_bt_v1.xml"
	POSE_PATH2="${PROJECT_SOURCE_DIR}/param/poses2.txt"
	POSE_PATHCHARGE="${PROJECT_SOURCE_DIR}/param/posesCharge.txt"
        	AW="Yi"
)







add_executable(mbf_bt_v2 src/mbf_bt_v2.cpp)
target_include_directories(mbf_bt_v2 PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(mbf_bt_v2 ${behaviortree_cpp_v3_LIBRARIES} ${catkin_LIBRARIES})
target_compile_definitions(mbf_bt_v2
        PRIVATE

	BT_XML_PATHA="${PROJECT_SOURCE_DIR}/param/BT_XML_PATHA.xml"
        POSE_PATHA="${PROJECT_SOURCE_DIR}/param/posesA.txt"
        
	BT_XML_PATHB="${PROJECT_SOURCE_DIR}/param/BT_XML_PATHB.xml"
        POSE_PATHB="${PROJECT_SOURCE_DIR}/param/posesB.txt"
        
	BT_XML_PATHC="${PROJECT_SOURCE_DIR}/param/BT_XML_PATHC.xml"
        POSE_PATHC="${PROJECT_SOURCE_DIR}/param/posesC.txt"
        
	BT_XML_PATHWarning="${PROJECT_SOURCE_DIR}/param/BT_XML_PATHWarning.xml"
        POSE_PATHWarning="${PROJECT_SOURCE_DIR}/param/posesWarning.txt"
        	AW="Yi"
        	
)

add_executable(mbf_client_node2 src/mbf_client_node2.cpp)
target_include_directories(mbf_client_node2 PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(mbf_client_node2 ${catkin_LIBRARIES})
target_compile_definitions(mbf_client_node2
        PRIVATE
        POSE_PATH2="${PROJECT_SOURCE_DIR}/param/poses2.txt"
)

add_executable(mbf_single_mode src/mbf_single_mode.cpp)
target_include_directories(mbf_single_mode PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(mbf_single_mode ${behaviortree_cpp_v3_LIBRARIES} ${catkin_LIBRARIES})
target_compile_definitions(mbf_single_mode
        PRIVATE

	BT_XML_PATH2="${PROJECT_SOURCE_DIR}/param/mbf_bt_v1.xml"
        POSE_PATH2="${PROJECT_SOURCE_DIR}/param/poses2.txt"
        	AW="Yi"
)

