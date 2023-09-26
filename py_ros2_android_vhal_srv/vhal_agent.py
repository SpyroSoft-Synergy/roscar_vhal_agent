#!/usr/bin/env python3

import rclpy
import gpiod
import time

from ros2_android_vhal.srv import SetVehicleProperty
from rclpy.node import Node


class X9C103:
    def __init__(self, cs_chip, cs_offset, inc_chip, inc_offset, ud_chip, ud_offset):
        self.resistance = 0
        self.chip = gpiod.Chip(cs_chip)

        self.cs_line = self.chip.get_line(cs_offset)
        self.inc_line = self.chip.get_line(inc_offset)
        self.ud_line = self.chip.get_line(ud_offset)

        self.cs_line.request(consumer='x9c103', type=gpiod.LINE_REQ_DIR_OUT)
        self.inc_line.request(consumer='x9c103', type=gpiod.LINE_REQ_DIR_OUT)
        self.ud_line.request(consumer='x9c103', type=gpiod.LINE_REQ_DIR_OUT)

        self.cs_line.set_value(1)
        self.inc_line.set_value(0)
        self.ud_line.set_value(0)

        self.shift_resistance(-100)

    def shift_resistance(self, steps):
        if steps > 0:
            self.ud_line.set_value(1)
        else:
            self.ud_line.set_value(0)

        self.cs_line.set_value(0)
        time.sleep(0.01)

        for _ in range(abs(steps)):
            self.inc_line.set_value(1)
            time.sleep(0.01)
            self.inc_line.set_value(0)
            time.sleep(0.01)

        self.cs_line.set_value(1)

    def set_resistance(self, steps):
        self.shift_resistance(steps - self.resistance)

    def cleanup(self):
        self.shift_resistance(-100)
        self.cs_line.release()
        self.inc_line.release()
        self.ud_line.release()


def step_for_voltage(v):
    if v > 12:
        v = 12
    return round(v/0.12)

class VehiclePropertyService(Node):

    def __init__(self, x9c103):
        super().__init__('ros2_android_vhal_service')
        print('Creating service')
        self.set_property_srv = self.create_service(SetVehicleProperty, 'SetVehicleProperty', self.set_vhal_property_callback)
        print('Service created')
        self.x9c103 = x9c103

    def set_vhal_property_callback(self, request, response):
        if request.prop.prop_id != 290459441:
            self.get_logger().info('Set property: %d \n' % (request.prop.prop_id))

            # HVAC_FAN_SPEED
            if request.prop.prop_id == 0x15400500:
                self.handle_fan_speed(request.prop)

        response.result = True
        return response

    def handle_fan_speed(self, request):
        if request.int32_values:
            volts = request.int32_values[0] * 2
            steps = round(volts/0.12)
            self.x9c103.set_resistance(steps)



def main():
    try:  
        rclpy.init()
        x9c103 = X9C103(cs_chip='gpiochip0', cs_offset=43, inc_chip='gpiochip0', inc_offset=42, ud_chip='gpiochip0', ud_offset=55)
        service = VehiclePropertyService(x9c103)
        rclpy.spin(service)

    except KeyboardInterrupt:
        x9c103.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
