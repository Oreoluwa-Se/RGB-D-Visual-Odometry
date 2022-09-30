set(node_plugins "")

# datareader components
add_library(reader_component SHARED ./dataset/reader.cpp)
target_compile_definitions(reader_component PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(reader_component my_slam)
ament_target_dependencies(reader_component std_msgs rclcpp rclcpp_components sensor_msgs cv_bridge nav_msgs)

rclcpp_components_register_nodes(reader_component dataset::Reader)
set(node_plugins "${node_plugins}dataset::Reader;
$<TARGET_FILE:reader_component>\n")

# viewer components
add_library(viewer_component SHARED ./control/viewer.cpp)
target_compile_definitions(viewer_component PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(viewer_component my_slam)
ament_target_dependencies(viewer_component rclcpp rclcpp_components cv_bridge sensor_msgs pcl_conversions)

rclcpp_components_register_nodes(reader_component my_slam::Viewer)
set(node_plugins "${node_plugins}my_slam::Viewer;
$<TARGET_FILE:viewer_component>\n")

# creating components for frontend
add_library(frontend_component SHARED ./control/frontend.cpp)
target_compile_definitions(frontend_component PRIVATE
							"COMPOSITION_BUILDING_DLL")
target_link_libraries(frontend_component 
						my_slam viewer_component) 
ament_target_dependencies(frontend_component
	rclcpp rclcpp_components geometry_msgs 
	tf2 nav_msgs tf2_ros)

rclcpp_components_register_nodes(frontend_component my_slam::Frontend)
set(node_plugins "${node_plugins}my_slam::Frontend;
$<TARGET_FILE:frontend_component>\n")

# creating components for visual odom
add_library(vio_component SHARED ./control/vio.cpp)
target_compile_definitions(vio_component PRIVATE
							"COMPOSITION_BUILDING_DLL")
ament_target_dependencies(vio_component rclcpp rclcpp_components)
target_link_libraries(vio_component 
						reader_component 
						frontend_component
						viewer_component 
						my_slam) 

rclcpp_components_register_nodes(vio_component "my_slam::VisualOdom")
set(node_plugins "${node_plugins}my_slam::VisualOdom;
$<TARGET_FILE:vio_component>\n")


# run odom components
add_executable(run_odom ./run_odom.cpp)
ament_target_dependencies(run_odom rclcpp)
target_link_libraries(run_odom
						reader_component 
						vio_component
						frontend_component
						viewer_component
						my_slam)

install(TARGETS
	my_slam
	reader_component
	frontend_component
	vio_component
	viewer_component

	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION BIN)

# executable install
install(TARGETS
		run_odom
		DESTINATION lib/${PROJECT_NAME})