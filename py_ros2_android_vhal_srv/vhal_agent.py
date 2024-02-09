#!/usr/bin/env python3

import rclpy
import gpiod
import threading
import time

from ros2_android_vhal.srv import SetVehicleProperty
from rclpy.node import Node


class PWMCtrl:
    def __init__(self, gpio_chip, pwm_line, frequency):
        self.chip = gpiod.Chip(gpio_chip)
        self.pwm_line = self.chip.get_line(pwm_line)
        self.pwm_line.request(consumer="VHAL Agent", type=gpiod.LINE_REQ_DIR_OUT)
        self.pwm_line.set_value(0)

        duty_cycle = 0.3
        self.period = 1.0 / frequency
        self.high_time = self.period * duty_cycle
        self.low_time = self.period - self.high_time
        self.running = True
        self.lock = threading.Lock()

        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def stop(self):
        self.running = False
        #if self.thread.is_alive():
        #    self.thread.join()

    def run(self):
        while self.running:
            with self.lock:
                # Sync the values
                current_running = self.running
                current_high_time = self.high_time
                current_low_time = self.low_time

            if self.high_time > 0:
                self.pwm_line.set_value(1)
                time.sleep(self.high_time)

            self.pwm_line.set_value(0)
            time.sleep(self.low_time)

    def set_speed(self, fan_speed):
        if fan_speed < 1 or fan_speed > 6:
            print(f'set_fan_speed error - invalid value: {fan_speed}')
        else:
            print(f'set_fan_speed: {fan_speed}')
            duty_cycle = (fan_speed - 1) * 0.2
            self.high_time = self.period * duty_cycle
            self.low_time = self.period - self.high_time

    def cleanup(self):
        self.stop()
        self.pwm_line.set_value(0)
        self.pwm_line.release()


class VehiclePropertyService(Node):

    def __init__(self, pwm_ctrl):
        super().__init__('ros2_android_vhal_service')
        print('Creating service')
        self.set_property_srv = self.create_service(SetVehicleProperty, 'set_vehicle_property', self.set_vhal_property_callback)
        print('Service created')
        self.pwm_ctrl = pwm_ctrl

    def set_vhal_property_callback(self, request, response):
        if request.prop.prop_id != 290459441:
            self.get_logger().info('Set property: %d \n' % (request.prop.prop_id))

            # HVAC_FAN_SPEED
            if request.prop.prop_id == 356517120:
                self.handle_fan_speed(request.prop)

        response.result = True
        return response

    def handle_fan_speed(self, request):
        if request.int32_values:
            print(f'handle_fan_speed: {request.int32_values[0]}')
            self.pwm_ctrl.set_speed(request.int32_values[0])



def main():
    try:  
        rclpy.init()
        fan_ctrl = PWMCtrl(gpio_chip='gpiochip0', pwm_line=23, frequency=70)
        service = VehiclePropertyService(fan_ctrl)
        rclpy.spin(service)

    except KeyboardInterrupt:
        fan_ctrl.cleanup()
        #rclpy.shutdown()

if __name__ == '__main__':
    main()
