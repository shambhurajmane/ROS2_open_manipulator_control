#!/usr/bin/env python3


"""
spawn_turtlebot.py

Script used to spawn a turtlebot in a generic position
"""
import os
import sys
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity

def main():
    """ Main for spwaning turtlebot node """
    # Get input arguments from user
    argv = sys.argv[1:]
    
    robot_name = "open_manipulator_x"
    package_name = robot_name + "_description"
    
    urdf_file = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        robot_name + ".urdf.xacro")
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    

    # Start node
    rclpy.init()
    node = rclpy.create_node("entity_spawner")

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")

    node.get_logger().info("Connecting to `/spawn_entity` service...")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...connected!")




    # Set data for request
    request = SpawnEntity.Request()
    request.name = "open_manipulator_x"
    request.xml = robot_desc
    request.robot_namespace = ""
    request.initial_pose.position.x = 0.0
    request.initial_pose.position.y = 0.0
    request.initial_pose.position.z = 0.0
    request.initial_pose.orientation.x = 0.0
    request.initial_pose.orientation.y = 0.0
    request.initial_pose.orientation.z = 0.1
    request.initial_pose.orientation.w = 0.0

    node.get_logger().info("Sending service request to `/spawn_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()