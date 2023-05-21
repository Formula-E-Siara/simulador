import time
from ament_index_python.packages import get_package_share_directory
import rclpy
from std_srvs.srv import Trigger
from ackermann_msgs.msg import AckermannDriveStamped
from pyPS4Controller.controller import Controller

from eufs_msgs.msg import CanState
from eufs_msgs.srv import SetCanState

import threading

class MyController(Controller):

    def __init__(self, interface, connecting_using_ds4drv=True, event_definition=None, event_format=None, pub=None, node=None, set_mission_cli=None, reset_srv=None):
        
        self.pub = pub
        self.node = node
        self.set_mission_cli = set_mission_cli
        self.reset_srv = reset_srv
        self.acc = 0.0
        self.x = 0
        self.y = 0
        self.state = 0
        
        Controller.__init__(self, interface, connecting_using_ds4drv=True, event_definition=None, event_format=None)

    def on_L3_left(self, value):
        self.x = value
        send_topic(self.pub , self.x, self.y, self.acc)

    def on_L3_right(self, value):
        self.x = value
        send_topic(self.pub , self.x, self.y, self.acc)

    # TODO: tem que fazer um polling aqui, ta dando errado porque so funciona no press e release
    # so registrar o press, cancelar no release
    def on_x_press(self):
        self.acc = 1.0
        send_topic(self.pub , self.x, self.y, self.acc)
    
    def on_x_release(self):
        self.acc = 0.0
        send_topic(self.pub , self.x, self.y, self.acc)

    def on_circle_press(self):
        self.state = CanState.AS_EMERGENCY_BRAKE
        update_mission_state(self.state, self.set_mission_cli, self.reset_srv)
        send_topic(self.pub , self.x, self.y, self.acc)

    def on_options_press(self):
        self.state = CanState.AMI_JOYSTICK
        update_mission_state(self.state, self.set_mission_cli, self.reset_srv)
        send_topic(self.pub , self.x, self.y, self.acc)    

    def on_L3_x_at_rest(self):
        self.x = 0
        send_topic(self.pub , self.x, self.y, self.acc)

    def on_L3_y_at_rest(self):
        self.y = 0
        send_topic(self.pub , self.x, self.y, self.acc)


def create_publishers():
    if not rclpy or not rclpy.ok():
        rclpy.init()

    node = rclpy.create_node('ds4input')
    pub_steer = node.create_publisher(AckermannDriveStamped, '/cmd', 10)

    mission_msg = CanState()
    mission_msg.ami_state = CanState.AMI_NOT_SELECTED


    request = SetCanState.Request()
    request.ami_state = mission_msg.ami_state

    set_mission_cli = node.create_client(SetCanState, 'ros_can/set_mission')
    reset_srv = node.create_client(Trigger, '/ros_can/reset')


    result = set_mission_cli.call_async(request)

    return node, pub_steer, set_mission_cli, reset_srv

def send_topic(pub, x, y, acc):
    drive = AckermannDriveStamped()
    drive.drive.steering_angle = -(x / 32767.0)    
    drive.drive.speed = 0.0
    drive.drive.acceleration = acc
    drive.drive.steering_angle_velocity = 0.0

    pub.publish(drive)

def update_mission_state(ami_state, set_mission_cli, reset_srv):

    request = Trigger.Request()
    result = reset_srv.call_async(request)

    mission_msg = CanState()
    mission_msg.ami_state = ami_state

    request = SetCanState.Request()
    request.ami_state = mission_msg.ami_state

    result = set_mission_cli.call_async(request)

def read_controller(controller):
    print('Hi from ds4input.')
    if not controller:
        return
    controller.listen()

def main():
    rclpy.init()

    mission_node, steering_publisher, set_mission_cli, reset_srv = create_publishers()

    controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False, pub=steering_publisher, node=mission_node, set_mission_cli=set_mission_cli, reset_srv=reset_srv)

    while True:
        read_controller(controller=controller)
        

if __name__ == '__main__':
    main()
