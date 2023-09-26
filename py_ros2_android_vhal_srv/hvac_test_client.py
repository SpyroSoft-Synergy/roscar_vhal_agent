#!/usr/bin/env python3

import sys
import rclpy
import time

from ros2_android_vhal.srv import SetVehicleProperty
from rclpy.node import Node


class VehiclePropertyclient(Node):

    def __init__(self):
        super().__init__('ros2_android_vhal_client')
        self.set_property_client = self.create_client(SetVehicleProperty, 'SetVehicleProperty')
        while not self.set_property_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting...')
        
        self.req = SetVehicleProperty.Request()

    def send_request(self, value):
        if value < 0 or value > 5:
            self.get_logger().error('Invalid request value, must be 0-5')
        self.req.prop.timestamp = 0
        self.req.prop.area_id = 0
        self.req.prop.prop_id = 0x15400500
        self.req.prop.int32_values = [value]
        self.future = self.set_property_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future, timeout_sec=2)
        return self.future.result()

def main():
    try:  
        rclpy.init()
        hvac_client = VehiclePropertyclient()
        response = hvac_client.send_request(int(sys.argv[1]))
        hvac_client.get_logger().info(f'Send request {int(sys.argv[1])}')

        time.sleep(1)
        
        hvac_client.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
