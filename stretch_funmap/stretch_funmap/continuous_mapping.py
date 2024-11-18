#!/usr/bin/env python3
import rclpy.node
from std_srvs.srv import Trigger

class ContinuousMapping(rclpy.node.Node):

    def __init__(self):
        super().__init__('continuous_mapping_node')
        self.drive_client = self.create_client(Trigger, "/funmap/trigger_drive_to_scan")
        self.head_scan_client = self.create_client(Trigger, "/funmap/trigger_head_scan")

    def send_request(self, head_scan_flag):
        request = Trigger.Request()
        if head_scan_flag:
            while not self.head_scan_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            
            self.get_logger().info('head scan')
            response = self.head_scan_client.call_async(request)
            rclpy.spin_until_future_complete(self, response)
            return response
        else:
            while not self.drive_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')

            self.get_logger().info('drive to scan')
            response = self.drive_client.call_async(request)
            rclpy.spin_until_future_complete(self, response)
            return response


def main():
    rclpy.init()

    node = ContinuousMapping()

    head_scan_flag = True
    while rclpy.ok():
        response = node.send_request(head_scan_flag)
        if response.result().success:
            head_scan_flag = not head_scan_flag
        else:
            node.get_logger().info("finished mapping")
            break

    # SAVE MAP HERE

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()