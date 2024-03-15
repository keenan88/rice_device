import rclpy
from rclpy.node import Node
from gpiozero import DigitalOutputDevice
from time import sleep
from geometry_msgs.msg import Twist
from math import pi
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time


usleep = lambda x: sleep(x/1000000.0)


class StepperDriver(Node):

    def __init__(self):
        super().__init__('Stepper_Driver')

        self.wheel_horiz_offset_m = 0.1 # Need to validate this
        self.wheel_radius_m = 101.8/1000/2
        self.lin_max_meters_per_s = 0.15
        self.ang_max_rad_per_s = pi / 2

        self.vel_sub = self.create_subscription(Twist, '/cmd_vel', self.set_motors_degrees_per_s, 10)
        self.odom_publisher = self.create_publisher(Odometry, '/wheel_odom', 10)
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = 'world'
        self.odom_msg.child_frame_id = 'robot'
        
        self.right_step_timer = self.create_timer(1, self.right_step_callback) 
        self.left_step_timer = self.create_timer(1, self.left_step_callback) 

        self.left_stopped = False
        self.right_stopped = False

        self.right_step_pin = DigitalOutputDevice(pin = 23, active_high=True, initial_value=False)
        self.right_dir_pin = DigitalOutputDevice(pin = 24, active_high=True, initial_value=False)

        self.left_step_pin = DigitalOutputDevice(pin = 14, active_high=True, initial_value=False)
        self.left_dir_pin = DigitalOutputDevice(pin = 15, active_high=True, initial_value=False)

        self.lps = 0

        starting_velocity = Twist()
        self.set_motors_degrees_per_s(starting_velocity)
    
        usleep(2000) # Must sleep atleast 1ms on startup, page 8 of drv8834 datasheet

    def set_motors_degrees_per_s(self, twist_msg: Twist):
        if twist_msg.linear.x > self.lin_max_meters_per_s:
            twist_msg.linear.x = self.lin_max_meters_per_s
        if twist_msg.linear.x < -self.lin_max_meters_per_s:
            twist_msg.linear.x = -self.lin_max_meters_per_s

        if twist_msg.angular.z > self.ang_max_rad_per_s:
            twist_msg.angular.z = self.ang_max_rad_per_s
        if twist_msg.angular.z < -self.ang_max_rad_per_s:
            twist_msg.angular.z = -self.ang_max_rad_per_s

        # It is a little cheesy to just take the prescribed velocity and assume that it is 100% correct
        # However, steppers have no feedback, so this is about all we can do.
        # We could try moving this to the stepper-stepping callbacks, but that is much more complex to
        # Implement, and may not even be more accurate.
        # More realistically, will see what the error is when moving around and use that in the covariance matrix.
        curr_ns = self.get_clock().now().nanoseconds
        self.odom_msg.header.stamp = Time(sec = int(curr_ns / 10e8), nanosec = int(curr_ns % 10e8))
        self.odom_msg.twist.twist.linear.x = twist_msg.linear.x
        self.odom_msg.twist.twist.angular.z = twist_msg.angular.z
        self.odom_publisher.publish(self.odom_msg)

        linear_component_rad_per_s = twist_msg.linear.x / self.wheel_radius_m
        rotation_component_rad_per_s = twist_msg.angular.z * self.wheel_horiz_offset_m / self.wheel_radius_m 

        right_wheel_rad_per_s = - linear_component_rad_per_s - rotation_component_rad_per_s
        left_wheel_rad_per_s = linear_component_rad_per_s - rotation_component_rad_per_s

        right_wheel_deg_per_s = right_wheel_rad_per_s * 180 / pi
        left_wheel_deg_per_s = left_wheel_rad_per_s * 180 / pi

        degrees_per_full_step = 1.8
        step_size = 1/2
        degrees_per_pulse = degrees_per_full_step * step_size

        right_pulses_per_s = right_wheel_deg_per_s / degrees_per_pulse

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
            self.right_dir_pin.off()
        else:
            self.right_dir_pin.on()
        
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
            self.lps += 1
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