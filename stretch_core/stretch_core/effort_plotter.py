# Required Hack whenever importing cv2 (stretch_body does it implicitly). Details: https://forum.hello-robot.com/t/1047
import os; os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = '/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms/libqxcb.so'
from multiprocessing_plotter import NBPlot

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class PlotterNode(Node):
    def __init__(self):
        super().__init__('plotter_node')
        self.node_name = self.get_name()
        self.get_logger().info("{0} started".format(self.node_name))

        self.topic_name = '/wheel_effort'
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.accel_subscriber = self.create_subscription(JointState, self.topic_name, self.plot_callback, qos_policy)
        self.pl=NBPlot()
        
    def plot_callback(self, wheel_effort):
        pl.plot(wheel_effort.effort[0], wheel_effort.effort[1], wheel_effort.effort[2], wheel_effort.effort[3])

def main():
    rclpy.init()
    node = PlotterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('interrupt received, so shutting down')


if __name__ == '__main__':
    main()
