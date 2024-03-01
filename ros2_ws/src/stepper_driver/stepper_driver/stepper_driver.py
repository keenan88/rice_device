import rclpy
from rclpy.node import Node
from gpiozero import DigitalOutputDevice
from time import sleep
from geometry_msgs.msg import Twist
from math import pi



usleep = lambda x: sleep(x/1000000.0)


class StepperDriver(Node):

    def __init__(self):
        super().__init__('Stepper_Driver')

        self.asdf = self.create_subscription(Twist, '/cmd_vel', self.set_speed_cb, 10)
        
        self.right_step_timer = self.create_timer(1, self.right_step_callback) 
        self.left_step_timer = self.create_timer(1, self.left_step_callback) 

        starting_velocity = Twist()
        self.set_motor_degrees_per_s(starting_velocity)
        
        self.right_step_pin = DigitalOutputDevice(pin = 17, active_high=True, initial_value=False)
        self.right_dir_pin = DigitalOutputDevice(pin = 27, active_high=True, initial_value=False)

        self.left_step_pin = DigitalOutputDevice(pin = 14, active_high=True, initial_value=False)
        self.left_dir_pin = DigitalOutputDevice(pin = 15, active_high=True, initial_value=False)

        usleep(2000) # Must sleep atleast 1ms on startup, page 8 of drv8834 datasheet

    def set_motor_degrees_per_s(self, twist_msg: Twist):

        wheel_horiz_offset_m = 0.1 # Need to validate this
        wheel_radius_m = 101.8/1000/2

        linear_component_rad_per_s = twist_msg.linear.x / wheel_radius_m
        rotation_component_rad_per_s = twist_msg.angular.z * wheel_horiz_offset_m / wheel_radius_m 

        right_wheel_rad_per_s = - linear_component_rad_per_s - rotation_component_rad_per_s
        left_wheel_rad_per_s = linear_component_rad_per_s - rotation_component_rad_per_s

        right_wheel_deg_per_s = right_wheel_rad_per_s * 180 / pi
        left_wheel_deg_per_s = left_wheel_rad_per_s * 180 / pi

        degrees_per_full_step = 1.8
        step_size = 1/2
        degrees_per_pulse = degrees_per_full_step * step_size

        right_pulses_per_s = right_wheel_deg_per_s / degrees_per_pulse
        right_pulse_period_ns = 10e8 / right_pulses_per_s

        left_pulses_per_s = left_wheel_deg_per_s / degrees_per_pulse
        left_pulse_period_ns = 10e8 / left_pulses_per_s

        self.right_stepper_timer.timer_period_ns = right_pulse_period_ns
        self.left_stepper_timer.timer_period_ns = left_pulse_period_ns


    def step_callback(self):
        
        self.right_step_pin.on()
        usleep(20) # Minimum 1.9us high time, as per page 8 of drv8834 datasheet
        self.right_step_pin.off() 
        usleep(20) # Minimum 1.9us low time, as per page 8 of drv8834 datasheet

        



def main(args=None):
    rclpy.init(args=args)

    stepper_driver = StepperDriver()

    rclpy.spin(stepper_driver)

    stepper_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()