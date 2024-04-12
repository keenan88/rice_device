import rclpy
from std_msgs.msg import Bool
import RPi.GPIO as GPIO
from rclpy.node import Node
from time import sleep
from math import pi
from std_msgs.msg import String, Int32
from li_interface.action import MoveLIAction
from rclpy.action import ActionServer


class Flopper_Localizer(Node):

    def __init__(self):
        super().__init__('Flopper_Localizer')

        self.states = [
            [
                [1, 1, 1],
                [0, 1, 0],
                [1, 1, 1],
                [1, 0, 1]
            ],

            [
                [1, 0, 1],
                [1, 1, 1],
                [0, 1, 0],
                [1, 1, 1]
            ],

            [
                [1, 1, 1],
                [1, 0, 1],
                [1, 1, 1],
                [0, 1, 0]
            ],

            [
                [0, 1, 0],
                [1, 0, 1],
                [1, 0, 1],
                [1, 1, 1]
            ]
        ]

        self.sensors_states = [
            [-1, -1, -1],
            [-1, -1, -1],
            [-1, -1, -1],
            [-1, -1, -1]
        ]

        self.localizer_state = "on"

        self.curr_state_idx = 0

        self.create_timer(0.1, self.update_flopper_localization_state)

        # Hardware config
        self.pin_numbers = [
            [6, 26, 23],
            [12, 5, 25],
            [13, 19, 8],
            [14, 21, 7]
        ]

        GPIO.setmode(GPIO.BCM)

        for flopper_row in self.pin_numbers:

            for pin_num in flopper_row:
                if pin_num != -1:
                    GPIO.setup(pin_num, GPIO.IN, pull_up_down=GPIO.PUD_OFF)


        # ROS2 communication
        self.last_hex_flag_publisher = self.create_publisher(Int32, 'last_hex_flag', 10)

        self.localization_state_subscriber = self.create_subscription(Int32, 'flopper_localization_state', self.update_localization_state, 10)


    # Reading from hardware and extracting state done in same callback for now, easier for synchronization
    def update_flopper_localization_state(self):
        if self.localizer_state == "reset":
            self.curr_state_idx = 0
            self.localizer_state = "on"

        if self.localizer_state == "on":
            for flopper_row_idx in range(len(self.pin_numbers)):
                flopper_row = self.pin_numbers[flopper_row_idx]

                pin_num_idx = 0
                for pin_num in flopper_row:
                    if pin_num != -1:
                        self.sensors_states[flopper_row_idx][pin_num_idx] = GPIO.input(pin_num)
                        #print(GPIO.input(pin_num), end=', ')
                    else:
                        pass
                        print("-1", end=', ')

                    pin_num_idx += 1

                print()

            same_states_cnt = 0

            for flopper_row_idx in range(len(self.pin_numbers)):
                flopper_row = self.pin_numbers[flopper_row_idx]

                for pin_num_idx in range(len(flopper_row)):

                    sensor_state = self.sensors_states[flopper_row_idx][pin_num_idx]
                    expected_pin_state = self.states[self.curr_state_idx + 1][flopper_row_idx][pin_num_idx]

                    if sensor_state == expected_pin_state:
                        same_states_cnt += 1

            if same_states_cnt >= 11:
                self.curr_state_idx = self.curr_state_idx + 1

            print("Current state:", self.curr_state_idx)

            if self.curr_state_idx == 3:
                last_hex_flag = Int32(1)
                self.flopper_state_publisher.publish(last_hex_flag)


            print()
            print()

    def update_localization_state(self, msg: String):
        self.localizer_state = msg.data





    
        
def main(args=None):
    rclpy.init(args=args)

    flopper_localizer = Flopper_Localizer()

    rclpy.spin(flopper_localizer)

    flopper_localizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()