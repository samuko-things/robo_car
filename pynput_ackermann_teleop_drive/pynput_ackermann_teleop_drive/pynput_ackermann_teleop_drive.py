from operator import imod
import sys

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from ackermann_msgs.msg import AckermannDrive

from pynput.keyboard import Key, Listener




arg_msg = """
enter drive args in format <car vel> <steer angle in deg>
        """

def process_args_vel():
    cmd_speed = 0.4 # in m/s
    steer_angle = 0.43633 # rads => 25 deg
    try:
        if len(sys.argv) == 1:
            print(arg_msg)
            print("using default values")
            return cmd_speed, steer_angle
        else:
            print("using entered drive command values")
            cmd_speed = float(sys.argv[1])
            steer_angle = float(sys.argv[2])
            return cmd_speed, steer_angle
    except Exception as e:
        print(e)
        print(arg_msg)
        print("using default values")
        return cmd_speed, steer_angle
    

msg = """
------------------------------------------------
Drive and steer with arrow keys:

                [forward] 
                    |
  [steer-left] ------------ [steer-right]
                    |
                [reverse] 

stops when no arrow key is pressed
-------------------------------------------------


w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only steer angle by 10%

ALT to reset speed

CTRL-C to quit
        """







class AckermannTeleopDrive(Node):
    def __init__(self):
        super().__init__(node_name="pynput_ackermann_teleop_drive_node") # initialize the node name     

        self.speedBindings = {
            'w': (1.1, 1.0),
            'x': (.9, 1.0),
            'e': (1.0, 1.1),
            'c': (1.0, .9),
        }

        self.speed_ctrl_keys = ['w', 'x', 'e', 'c']

        self.max_angle = 1.2217 #rads =>70.0 deg
        self.min_angle = 0.08727 #rads =>5.0 deg

        self.default_speed, self.default_turn = process_args_vel()

        self.cmd_speed = self.default_speed
        self.steer_angle = self.default_turn

        self.v_dir = 0
        self.th_dir = 0

        self.can_print = True

        self.status = 0


        


        self.ackermannDriveCmd = AckermannDrive()

        self.send_cmd = self.create_publisher(AckermannDrive, 'cmd_ackermann', 10)

        # # the timer will automatically run the timer callback function
        # # for every timer_period
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.publish_cmd)

        # ...or, in a non-blocking fashion:
        listener = Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()
        # listener.join()

        print(msg)

    def publish_cmd(self):
        self.ackermannDriveCmd.steering_angle = self.th_dir*self.steer_angle
        self.ackermannDriveCmd._speed = self.v_dir*self.cmd_speed

        self.send_cmd.publish(self.ackermannDriveCmd)

        self.print_speed()

    def print_speed(self):
        if self.can_print:
            if (self.status == 14):
                print(msg)
            self.status = (self.status + 1) % 15

            print('currently:\tspeed=%s\tturn=%s' % (self.cmd_speed, self.steer_angle))
            self.can_print=False


    def reset_speed(self):
        self.cmd_speed = self.default_speed
        self.steer_angle = self.default_turn
        self.can_print=True

    def on_press(self, key):       
        if key == Key.up:
            if self.v_dir == 0:
                self.v_dir = 1
                
        elif key == Key.down:
            if self.v_dir == 0:
                self.v_dir = -1



        if key == Key.left:
            if self.th_dir == 0:
                self.th_dir = 1
                
        elif key == Key.right:
            if self.th_dir == 0:
                self.th_dir = -1


        if key == Key.alt:
            self.reset_speed() 

        
        if hasattr(key, 'char'):
            if key .char in self.speed_ctrl_keys:
                self.cmd_speed = self.cmd_speed * self.speedBindings[key.char][0]
                self.steer_angle = self.steer_angle * self.speedBindings[key.char][1]

                if self.steer_angle > self.max_angle:
                    self.steer_angle = self.max_angle
                if self.steer_angle < self.min_angle:
                    self.steer_angle = self.min_angle
                
                self.can_print=True


                    
    def on_release(self, key):

        if key == Key.up:
            if self.v_dir == 1:
                self.v_dir = 0
                
        elif key == Key.down:
            if self.v_dir == -1:
                self.v_dir = 0



        if key == Key.left:
            if self.th_dir == 1:
                self.th_dir = 0
                
        elif key == Key.right:
            if self.th_dir == -1:
                self.th_dir = 0


        if key == Key.esc:
            # Stop listener
            return False
    






def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the publisher node
    ackermannTeleop = AckermannTeleopDrive()

    # spin the node so the call back function is called
    rclpy.spin(ackermannTeleop)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ackermannTeleop.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown() 



if __name__=='__main__':
    main()

