#!/usr/bin/env python
import rospy
from sparse_gslam.msg import RawData
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from sensor_msgs.msg import Joy

import logging
import time
import threading
import math
import os
import functools
from concurrent.futures import ThreadPoolExecutor

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# only one packet is transmitted everytime, the size of the packet is ~30byte, the empirical result shows that up to 26 bytes can be transmitted (6 floats + 1 uint16_t)


class CrazyflieCommunicator:
    def __init__(self, stockFirmware=False):
        self.teleop = rospy.get_param("~teleop")
        self.hover_before_start = rospy.get_param("~hover_before_start")
        self.height = rospy.get_param("~height")

        self.twist = Twist()
        self.scf = SyncCrazyflie(rospy.get_param("~address"), cf=Crazyflie(rw_cache=os.path.join(
            os.path.dirname(__file__),
            "cache"
        )))
        # self.scf = SyncCrazyflie(uri, cf=Crazyflie())
        try:
            self.scf.open_link()
        except Exception as e:
            print("Exception: ", e)
            self.scf.close_link()
            return

        checks = self.decks_check([
            ("deck", "bcMultiranger"),
            ("deck", "bcFlow2")
        ])
        if not checks[0]:
            logging.error("Multiranger not attached")
        if not checks[1]:
            logging.error("Flowdeck not attached")
        
        self.state_pub1 = rospy.Publisher('state_xyzv', RawData, queue_size=1)
        self.state_pub2 = rospy.Publisher('state_ranger_qxyzw', RawData, queue_size=1)

        self.wait_lock = threading.Lock()
        self.wait = True
        
        self.lg_stab1 = self.lg_stab_config1()
        self.lg_stab2 = self.lg_stab_config2()

        self.scf.cf.log.add_config(self.lg_stab1)
        self.scf.cf.log.add_config(self.lg_stab2)

        self.lg_stab1.start()
        self.lg_stab2.start()

        if self.teleop:
            if not stockFirmware:
                self.set_param_sync("teleop", "teleop", 1)
            if self.hover_before_start:
                wait_thread = threading.Thread(target=self.hover_before_control)
                wait_thread.start()
            if rospy.get_param("~gamepad"):
                print("using gamepad")
                self.command_sub = rospy.Subscriber("joy", Joy, callback=self.gamepad_callback, queue_size=1)
            else:
                self.command_sub = rospy.Subscriber("cmd_vel", Twist, callback=self.control_callback, queue_size=1)
        else:
            assert stockFirmware == False
            self.set_param_sync("teleop", "nominal_height", self.height)
            self.takeoff_srv = rospy.Service('takeoff', SetBool, self.takeoff_srv_handler)

    def decks_check(self, groups_names):
        num_decks = len(groups_names)
        events = [threading.Event() for i in range(num_decks)]
        decks_attached = [False] * num_decks

        def callback(i, name, value_str):
            print(name, value_str)
            if int(value_str):
                decks_attached[i] = True
            events[i].set()

        for i, (group, name) in enumerate(groups_names):
            self.scf.cf.param.add_update_callback(group=group, name=name, cb=functools.partial(callback, i))

        for i, event, (group, name) in zip(range(num_decks), events, groups_names):
            event.wait()
            self.scf.cf.param.remove_update_callback(group=group, name=name)

        return decks_attached

    def set_param_async(self, group, name, value):
        event = threading.Event()
        def callback(name, value):
            print('Parameter ' + name + ' set to number: ' + value)
            event.set()
        self.scf.cf.param.add_update_callback(group, name, callback)
        self.scf.cf.param.set_value("{}.{}".format(group, name), value)
        return event
    
    def set_param_sync(self, group, name, value):
        self.set_param_async(group, name, value).wait()
        self.scf.cf.param.remove_update_callback(group, name)
        
    def takeoff_srv_handler(self, req: SetBoolRequest):
        self.set_param_sync("teleop", "keep_flying", req.data)
        return SetBoolResponse(True, "")
    
    def hover_before_control(self):
        while not rospy.is_shutdown():
            with self.wait_lock:
                if not self.wait:
                    break
            self.scf.cf.commander.send_hover_setpoint(0, 0, 0.0, self.height)
            time.sleep(0.1)

    def control_callback(self, data: Twist):
        if self.hover_before_start:
            with self.wait_lock:
                self.wait = False
            
            self.scf.cf.commander.send_hover_setpoint(
                data.linear.x, data.linear.y, math.degrees(data.angular.z), self.height
            )
            time.sleep(0.1)

    def gamepad_callback(self, data: Joy):
        # print(data)
        x_vel = data.axes[1] * 0.6
        y_vel = data.axes[0] * 0.6
        if data.buttons[0]:
            self.height -= 0.02
        if data.buttons[3]:
            self.height += 0.02
        self.scf.cf.commander.send_hover_setpoint(
            x_vel, y_vel, -data.axes[3] * 70, self.height
        )

    def lg_stab_config1(self):
        lg_stab = LogConfig(name='conf1', period_in_ms=100)
        # lg_stab.add_variable('stateEstimate.x', 'float')
        # lg_stab.add_variable('stateEstimate.y', 'float')
        # lg_stab.add_variable('stateEstimate.z', 'float')
        lg_stab.add_variable('stateEstimateZ.x', 'int16_t')
        lg_stab.add_variable('stateEstimateZ.y', 'int16_t')
        lg_stab.add_variable('stateEstimateZ.z', 'int16_t')

        lg_stab.add_variable('stateEstimateZ.vx', 'int16_t')
        lg_stab.add_variable('stateEstimateZ.vy', 'int16_t')
        lg_stab.add_variable('stateEstimateZ.vz', 'int16_t')

        lg_stab.add_variable('stateEstimateZ.rateRoll', 'int16_t')
        lg_stab.add_variable('stateEstimateZ.ratePitch', 'int16_t')
        lg_stab.add_variable('stateEstimateZ.rateYaw', 'int16_t')

        lg_stab.add_variable('pm.vbatMV', 'uint16_t')
        lg_stab.add_variable('radio.rssi', 'uint8_t')
        # lg_stab.add_variable('range.zrange', 'uint16_t')
        lg_stab.data_received_cb.add_callback(self.log_stab_callback1)
        return lg_stab

    def lg_stab_config2(self):
        lg_stab = LogConfig(name='conf2', period_in_ms=100)
        lg_stab.add_variable('stateEstimate.qw', 'float')
        lg_stab.add_variable('stateEstimate.qx', 'float')
        lg_stab.add_variable('stateEstimate.qy', 'float')
        lg_stab.add_variable('stateEstimate.qz', 'float')

        lg_stab.add_variable('range.front', 'uint16_t')
        lg_stab.add_variable('range.back', 'uint16_t')
        lg_stab.add_variable('range.left', 'uint16_t')
        lg_stab.add_variable('range.right', 'uint16_t')
        lg_stab.add_variable('range.up', 'uint16_t')
        lg_stab.data_received_cb.add_callback(self.log_stab_callback2)
        return lg_stab

    def log_stab_callback1(self, timestamp, data, logconf):
        msg = RawData()
        msg.header.stamp = rospy.Time.from_sec(timestamp * 0.001)
        msg.raw = [
            data['stateEstimateZ.x'] * 0.001,
            data['stateEstimateZ.y'] * 0.001,
            data['stateEstimateZ.z'] * 0.001,

            data['stateEstimateZ.vx'] * 0.001,
            data['stateEstimateZ.vy'] * 0.001,
            data['stateEstimateZ.vz'] * 0.001,

            data['stateEstimateZ.rateRoll'] * 0.001,
            data['stateEstimateZ.ratePitch'] * 0.001,
            data['stateEstimateZ.rateYaw'] * 0.001,

            data["pm.vbatMV"] * 0.001,
            data["radio.rssi"],
            # data["range.zrange"] * 0.001
        ]
        self.state_pub1.publish(msg)

    def log_stab_callback2(self, timestamp, data, logconf):
        msg = RawData()
        msg.header.stamp = rospy.Time.from_sec(timestamp * 0.001)
        msg.raw = [
            data['range.right'] * 0.001,
            data['range.front'] * 0.001,
            data['range.left'] * 0.001,
            data['range.back'] * 0.001,
            data['range.up'] * 0.001,
            data['stateEstimate.qx'],
            data['stateEstimate.qy'],
            data['stateEstimate.qz'],
            data['stateEstimate.qw']
        ]
        self.state_pub2.publish(msg)

    def dispose(self):
        if self.scf._is_link_open:
            if not self.teleop:
                self.set_param_sync("teleop", "keep_flying", False)
            self.lg_stab1.stop()
            self.lg_stab2.stop()
            self.scf.close_link()

if __name__ == '__main__':
    rospy.init_node('crazyflie_communicator')
    cflib.crtp.init_drivers(enable_debug_driver=False)
    time.sleep(1)

    handle = CrazyflieCommunicator()
    rospy.spin()
    handle.dispose()