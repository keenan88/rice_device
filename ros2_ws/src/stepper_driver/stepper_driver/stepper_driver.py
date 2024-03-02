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

        self.asdf = self.create_subscription(Twist, '/cmd_vel', self.set_motors_degrees_per_s, 10)
        
        self.right_step_timer = self.create_timer(1, self.right_step_callback) 
        self.left_step_timer = self.create_timer(1, self.left_step_callback) 

        self.left_stopped = False
        self.right_stopped = False

        self.right_step_pin = DigitalOutputDevice(pin = 17, active_high=True, initial_value=False)
        self.right_dir_pin = DigitalOutputDevice(pin = 27, active_high=True, initial_value=False)

        self.left_step_pin = DigitalOutputDevice(pin = 23, active_high=True, initial_value=False)
        self.left_dir_pin = DigitalOutputDevice(pin = 24, active_high=True, initial_value=False)

        starting_velocity = Twist()
        self.set_motors_degrees_per_s(starting_velocity)
    
        usleep(2000) # Must sleep atleast 1ms on startup, page 8 of drv8834 datasheet

    def set_motors_degrees_per_s(self, twist_msg: Twist):

        wheel_horiz_offset_m = 0.1 # Need to validate this
        wheel_radius_m = 101.8/1000/2

        lin_max_meters_per_s = 0.15
        ang_max_rad_per_s = pi / 2

        if twist_msg.linear.x > lin_max_meters_per_s:
            twist_msg.linear.x = lin_max_meters_per_s
        if twist_msg.linear.x < -lin_max_meters_per_s:
            twist_msg.linear.x = -lin_max_meters_per_s

        if twist_msg.angular.z > ang_max_rad_per_s:
            twist_msg.angular.z = ang_max_rad_per_s
        if twist_msg.angular.z < -ang_max_rad_per_s:
            twist_msg.angular.z = -ang_max_rad_per_s

        linear_component_rad_per_s = twist_msg.linear.x / wheel_radius_m
        rotation_component_rad_per_s = twist_msg.angular.z * wheel_horiz_offset_m / wheel_radius_m 

        right_wheel_rad_per_s = - linear_component_rad_per_s - rotation_component_rad_per_s
        left_wheel_rad_per_s = linear_component_rad_per_s - rotation_component_rad_per_s

        right_wheel_deg_per_s = right_wheel_rad_per_s * 180 / pi
        left_wheel_deg_per_s = left_wheel_rad_per_s * 180 / pi

        print("R deg/s:", round(right_wheel_deg_per_s, 2))
        print("L deg/s:", round(left_wheel_deg_per_s, 2))
        print()

        degrees_per_full_step = 1.8
        step_size = 1/2
        degrees_per_pulse = degrees_per_full_step * step_size

        right_pulses_per_s = right_wheel_deg_per_s / degrees_per_pulse
        print(right_pulses_per_s)

        if right_pulses_per_s != 0:
            self.right_stopped = False
            right_pulse_period_ns = 10e8 / abs(right_pulses_per_s)
            self.right_step_timer.timer_period_ns = right_pulse_period_ns
            
        else:
            self.right_stopped = True

        left_pulses_per_s = left_wheel_deg_per_s / degrees_per_pulse

        if left_pulses_per_s != 0:
            left_pulse_period_ns = 10e8 / abs(left_pulses_per_s)
            self.left_step_timer.timer_period_ns = left_pulse_period_ns
            self.left_stopped = False
        else:
            self.left_stopped = True

        if right_wheel_deg_per_s > 0:
            self.right_dir_pin.on()
            print("Dir pin off")
        else:
            self.right_dir_pin.off()
            print("Dir pin on")
        
        if left_wheel_deg_per_s > 0:
            self.left_dir_pin.on()
        else:
            self.left_dir_pin.off()

        


    def right_step_callback(self):

        if not self.right_stopped:        
            self.right_step_pin.on()
            usleep(20) # Minimum 1.9us high time, as per page 8 of drv8834 datasheet
            self.right_step_pin.off() 
            usleep(20) # Minimum 1.9us low time, as per page 8 of drv8834 datasheet

    def left_step_callback(self):
        
        if not self.left_stopped:

            self.left_step_pin.on()
            usleep(20) # Minimum 1.9us high time, as per page 8 of drv8834 datasheet
            self.left_step_pin.off() 
            usleep(20) # Minimum 1.9us low time, as per page 8 of drv8834 datasheet

        



def main(args=None):
    rclpy.init(args=args)

    stepper_driver = StepperDriver()

    rclpy.spin(stepper_driver)

    stepper_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()