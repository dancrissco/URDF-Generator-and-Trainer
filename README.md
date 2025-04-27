# URDF-Generator-and-Trainer
Mobile Differential Robot - 2 Wheels
Features
User input-driven URDF creation

Customizable:

Base link dimensions (length, width, height)

Wheel radius and width

Wheel spacing

Optional caster wheel

Clean URDF structure compatible with ROS 2, RViz, and robot_state_publisher

Lightweight and easily expandable for more complex robot designs

How It Works
When you run the script, it interactively asks for key robot parameters.
Defaults are provided for quick testing, but you can customize any aspect to match your robot's design.

Example:

python3 urdf_gen_with_inputs.py

You can immediately visualize the generated URDF by running:

ros2 run robot_state_publisher robot_state_publisher generated_robot.urdf

rviz2

ros2 run joint_state_publisher_gui joint_state_publisher_gui
