import rclpy
from rclpy.node import Node

from gpiozero import DigitalOutputDevice
from time import sleep

usleep = lambda x: sleep(x/1000000.0)


class StepperDriver(Node):

    def __init__(self):
        super().__init__('AccelExtractor')

        timer_period_s = 0.01

        self.vel_subscriber = self.create_subscription('/motor_speed', self.set_speed_cb, 10)

        self.step_size = 0.5
        self.degrees_per_step = 1.8

        self.turn_steppers = self.create_timer(timer_period_s, self.step_callback) 

        self.step_pin = DigitalOutputDevice(pin = 17, active_high=True, initial_value=False)

        self.i = 0

        usleep(2000) # Must sleep atleast 1ms on startup, page 8 of drv8834 datasheet

    def step_callback(self):
        self.i += 1
        print(self.i)  

        self.step_pin.on()
        usleep(20) # Minimum 1.9us high time, as per page 8 of drv8834 datasheet
        self.step_pin.off() 
        usleep(20) # Minimum 1.9us low time, as per page 8 of drv8834 datasheet



def main(args=None):
    rclpy.init(args=args)

    stepper_driver = StepperDriver()

    rclpy.spin(stepper_driver)

    stepper_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()