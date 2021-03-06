cmake_minimum_required(VERSION 2.8.3)
project(coop_pkg)

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs	
	roscpp	
	message_generation
	genmsg
	std_msgs
)

add_message_files(
	FILES
	Robot.msg
	Task.msg
	Resource.msg
	Skill.msg
)

add_service_files(
	FILES
	AddSkill.srv
	SetBusy.srv
)

generate_messages(
	DEPENDENCIES
	geometry_msgs
	std_msgs
)

catkin_package()

include_directories(include ${catkin_INLCUDE_DIRS})

add_executable(coop_node 
		src/coop_node.cpp
		src/Robot.cpp
		src/Task.cpp		
		src/Coop.cpp
		src/Resource.cpp
		src/Skill.cpp
		src/RobotsPerTask.cpp
		)

target_link_libraries(coop_node
   	${catkin_LIBRARIES}
)

add_dependencies(coop_node 
	${catkin_EXPORTED_TARGETS}
)

add_executable(robot_node
		src/robot_node.cpp
		src/RobotNode.cpp
		src/Robot.cpp
		src/Resource.cpp
		src/Skill.cpp
		src/Task.cpp
		)

target_link_libraries(robot_node
   	${catkin_LIBRARIES}
)

add_dependencies(robot_node 
	${catkin_EXPORTED_TARGETS}
)

add_executable(task_node
		src/task_node.cpp
		src/Task.cpp
		src/Resource.cpp
		src/Skill.cpp
		)

target_link_libraries(task_node
   	${catkin_LIBRARIES}
)

add_dependencies(task_node 
	${catkin_EXPORTED_TARGETS}
)
