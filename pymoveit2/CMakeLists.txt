cmake_minimum_required(VERSION 3.5)
project(pymoveit2)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})


# Install examples
set(EXAMPLES_DIR examples)
install(PROGRAMS
    ${EXAMPLES_DIR}/ex_collision_object.py
    ${EXAMPLES_DIR}/ex_gripper_command.py
    ${EXAMPLES_DIR}/ex_gripper.py
    ${EXAMPLES_DIR}/ex_joint_goal.py
    ${EXAMPLES_DIR}/ex_pose_goal.py
    ${EXAMPLES_DIR}/ex_servo.py
    ${EXAMPLES_DIR}/pick_goal.py
    ${EXAMPLES_DIR}/testex_pose_goal.py
    ${EXAMPLES_DIR}/robot2_main.py
    ${EXAMPLES_DIR}/test_code.py
    DESTINATION lib/${PROJECT_NAME}
)

# Install textilePrograms
set(textilePrograms_DIR textilePrograms)
install(PROGRAMS
    ${textilePrograms_DIR}/io_port_toggle.py
    ${textilePrograms_DIR}/place_cotton.py
    ${textilePrograms_DIR}/pick_goal.py
    ${textilePrograms_DIR}/joint_goal.py
    ${textilePrograms_DIR}/robotSubscriber.py
    ${textilePrograms_DIR}/place_mix.py
    ${textilePrograms_DIR}/place_wool.py
    ${textilePrograms_DIR}/robotTestPublisher.py
    ${textilePrograms_DIR}/main.py
    ${textilePrograms_DIR}/testSub.py
    ${textilePrograms_DIR}/testPub.py
    ${textilePrograms_DIR}/find_joint_angles.py
    ${textilePrograms_DIR}/ex_pose_goal.py

    DESTINATION lib/${PROJECT_NAME}
)

# Setup the project
ament_package()
