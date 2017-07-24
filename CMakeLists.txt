cmake_minimum_required(VERSION 2.8.3)
project(bu_project)

find_package(catkin REQUIRED COMPONENTS cmake_modules apriltags  cv_bridge image_transport message_generation sensor_msgs)
find_package(Eigen REQUIRED)

add_message_files(DIRECTORY msg
   FILES ResizedImage.msg
         tagdata3d.msg
	 filteredaverage.msg
   )

generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

catkin_package(INCLUDE_DIRS include
           LIBRARIES apriltags
           CATKIN_DEPENDS apriltags cv_bridge image_transport message_runtime sensor_msgs
           DEPENDS Eigen)

find_package(OpenCV)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_executable(my_publisher src/my_publisher.cpp)
add_dependencies(my_publisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(my_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(tag_calculations src/tag_calculations.cpp)
add_dependencies(tag_calculations ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(tag_calculations ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(noise_variance_sampling src/noise_variance_sampling.cpp)
add_dependencies(noise_variance_sampling ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(noise_variance_sampling ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(filtered_average src/filtered_average.cpp)
add_dependencies(filtered_average ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(filtered_average ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(my_subscriber src/my_subscriber.cpp)
add_dependencies(my_subscriber ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(my_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS my_publisher my_subscriber
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

install(FILES resized_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

