import rclpy
from rclpy.node import Node
from gpiozero import DigitalOutputDevice
from time import sleep
from std_msgs.msg import String



usleep = lambda x: sleep(x/1000000.0)


class StepperDriver(Node):

    def __init__(self):
        super().__init__('AccelExtractor')

        self.asdf = self.create_subscription(String, '/motor_speed', self.set_speed_cb, 10)
        
        self.stepper_timer = self.create_timer(1, self.step_callback) 
        self.set_motor_degrees_per_s(30)
        
        self.step_pin = DigitalOutputDevice(pin = 17, active_high=True, initial_value=False)

        usleep(2000) # Must sleep atleast 1ms on startup, page 8 of drv8834 datasheet

        print("period (ns):", self.stepper_timer.timer_period_ns)

    def set_motor_degrees_per_s(self, degrees_per_s):
        degrees_per_full_step = 1.8
        step_size = 1/2
        degrees_per_pulse = degrees_per_full_step * step_size
        pulses_per_s = degrees_per_s / degrees_per_pulse
        pulse_period_ns = 10e8 / pulses_per_s

        self.stepper_timer.timer_period_ns = pulse_period_ns
        
        return pulse_period_ns

    def step_callback(self):
        
        self.step_pin.on()
        usleep(20) # Minimum 1.9us high time, as per page 8 of drv8834 datasheet
        self.step_pin.off() 
        usleep(20) # Minimum 1.9us low time, as per page 8 of drv8834 datasheet

    def set_speed_cb(self, desired_deg_per_s: String):
        self.set_motor_degrees_per_s(float(desired_deg_per_s.data))
        



def main(args=None):
    rclpy.init(args=args)

    stepper_driver = StepperDriver()

    rclpy.spin(stepper_driver)

    stepper_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()