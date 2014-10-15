cmake_minimum_required(VERSION 2.8.3)
project(pr2_sith)
find_package(catkin REQUIRED COMPONENTS rospy actionlib pr2_controllers_msgs pr2_gripper_sensor_action pr2_gripper_sensor_controller pr2_gripper_sensor_msgs pr2_precise_trajectory sensor_msgs dynamic_reconfigure)

generate_dynamic_reconfigure_options(cfg/TheForce.cfg)

catkin_package(
    DEPENDS rospy actionlib pr2_controllers_msgs pr2_gripper_sensor_action pr2_gripper_sensor_controller pr2_gripper_sensor_msgs pr2_precise_trajectory sensor_msgs dynamic_reconfigure
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(DIRECTORY cfg/ sounds/ scripts/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES duel_auto.launch duel_core.launch duel_no_setup.launch duel_with_setup.launch pr2_sith.app pr2_sith.interface pr2_sith.png DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

