import json
import logging
import os

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

FILENAME = os.path.join(os.getcwd(), "src/getpose/resource/amclpose.log")
LOGFORMAT = "%(message)s"


class GetPoseNode(Node):
    """Collect the pose data of the rover."""
    def __init__(self, logger: logging.Logger):
        super().__init__("getpose")
        self.log = logger
        self.sub = self.create_subscription(
            msg_type=PoseWithCovarianceStamped,
            topic="amcl_pose",
            callback=self.write_to_log,
            qos_profile=qos_profile_system_default,
        )

    def write_to_log(self, msg: PoseWithCovarianceStamped):
        """
        Convert message into JSON format and write position into a log file.

        :param msg: Timestamped pose message.

        """
        #self.get_logger().info(msg)
        ts = float(f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec}")
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        data = {"ts": ts, "px": px, "py": py, "pz": pz, "qt": [qx, qy, qz, qw]}
        self.log.debug(json.dumps(data))


def main():
    logging.basicConfig(filename=FILENAME, format=LOGFORMAT, level=logging.DEBUG)
    logger = logging.getLogger()

    rclpy.init()
    
    # Set an initial pose before collecting the pose data.
    navigator = TurtleBot4Navigator()
    if not navigator.getDockedStatus():
        navigator.info("Docking before initialising pose")
        navigator.dock()
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    print("Collecting the pose data...")
    node = GetPoseNode(logger)
    try:
        rclpy.spin(node)
    except ExternalShutdownException as err:
        print(err)
    except KeyboardInterrupt:
        print("Shutting down the node...")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
