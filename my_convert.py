import rosbag
import rospy
from std_msgs.msg import Int32, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import math
import tf
from tf2_msgs.msg import TFMessage
import numpy as np

def convert_carmen(fpath, out_name):
    bag = rosbag.Bag('src/sparse_gslam/bags/%s.bag' % out_name, 'w')

    with open(fpath, "r") as f:
        message = []
        for line in f.readlines():
            if line.startswith("FLASER"):
                line = line.split(" ")[1:]

                laser_scan = LaserScan()
                laser_scan.header.frame_id = "base_link"
                laser_scan.angle_min = math.radians(-90)
                laser_scan.angle_max = math.radians(90)
                laser_scan.range_min = 0.0
                laser_scan.range_max = 4.0
                num_readings = int(line[0])
                laser_scan.angle_increment = math.pi / num_readings
                for i in range(1, num_readings + 1):
                    f = float(line[i])
                    if f >= 10:
                        laser_scan.ranges.append(float('nan'))
                    else:
                        laser_scan.ranges.append(f)
                i += 1

                trans = TransformStamped()
                trans.header.frame_id = "odom"
                trans.child_frame_id = "base_link"
                trans.transform.translation.x = float(line[i+3])
                trans.transform.translation.y = float(line[i+4])
                trans.transform.translation.z = 0
                
                quat = tf.transformations.quaternion_from_euler(0.0, 0.0, float(line[i+5]))
                trans.transform.rotation.x = quat[0]
                trans.transform.rotation.y = quat[1]
                trans.transform.rotation.z = quat[2]
                trans.transform.rotation.w = quat[3]

                timestamp = float(line[i+6])
                trans.header.stamp = laser_scan.header.stamp = rospy.Time(timestamp)
                
                tf_msg = TFMessage()
                tf_msg.transforms.append(trans)
                message.append((timestamp, tf_msg, laser_scan))
        
        message.sort(key=lambda x: x[0])
        print(len(message))
        for i, (_, tf_msg, laser_scan) in enumerate(message):
            tf_msg.transforms[0].header.seq = i
            laser_scan.header.seq = i
            bag.write("tf", tf_msg, laser_scan.header.stamp)
            bag.write("scan", laser_scan, laser_scan.header.stamp)

    bag.close()

def convert_carmen_bspline(fpath, out_name):
    with open(fpath, "r") as f:
        with open(out_name, 'w') as f2:
            for line in f.readlines():
                if line.startswith("FLASER"):
                    line = line.split(" ")[1:]
                    message = []
                    timestamp = line[-1].strip()
                    num_readings = int(line[0])
                    laser_scan = []
                    for i in range(1, num_readings + 1):
                        laser_tmp = line[i]
                        laser_scan.append(laser_tmp)
                    i += 1
                    odomx = line[i+3]
                    odomy = line[i+4]
                    odomt = line[i+5]

                    message.append(timestamp)
                    message.append(odomx)
                    message.append(odomy)
                    message.append(odomt)
                    message.extend(laser_scan)

                    message = " ".join(message)
                    message = message + "\n"
                    f2.write(message)


def convert_radish(fpath, out_name):
    bag = rosbag.Bag('src/sparse_gslam/bags/%s.bag' % out_name, 'w')

    with open(fpath, "r") as f:
        seq = 0
        
        for line in f.readlines():
            if line.startswith("position"):
                line = line.split(" ")[3:]
                last_x = float(line[0])
                last_y = float(line[1])
                last_theta = float(line[2])

            elif line.startswith("laser"):
                line = line.split(" ")
                tstamp = float(line[2])
                line = line[3:-1]

                laser_scan = LaserScan()
                laser_scan.header.frame_id = "base_link"
                laser_scan.angle_min = math.radians(-90)
                laser_scan.angle_max = math.radians(90)
                laser_scan.angle_increment = math.radians(1)
                laser_scan.range_min = 0.0
                laser_scan.range_max = 4.0
                num_readings = 181
                for i in range(num_readings):
                    laser_scan.ranges.append(float(line[3 * i]))

                trans = TransformStamped()
                trans.header.frame_id = "odom"
                trans.child_frame_id = "base_link"
                trans.transform.translation.x = last_x
                trans.transform.translation.y = last_y
                trans.transform.translation.z = 0
                
                quat = tf.transformations.quaternion_from_euler(0.0, 0.0, last_theta)
                trans.transform.rotation.x = quat[0]
                trans.transform.rotation.y = quat[1]
                trans.transform.rotation.z = quat[2]
                trans.transform.rotation.w = quat[3]

                trans.header.stamp = laser_scan.header.stamp = rospy.Time(tstamp)
                trans.header.seq = laser_scan.header.seq = seq
                
                tf_msg = TFMessage()
                tf_msg.transforms.append(trans)
                bag.write("tf", tf_msg, trans.header.stamp)
                bag.write("scan", laser_scan, trans.header.stamp)
                seq += 1

    bag.close()


def convert_stanford_gates(fpath, out_name):
    bag = rosbag.Bag('src/sparse_gslam/bags/%s.bag' % out_name, 'w')

    with open(fpath, "r") as f:
        seq = 0
        last_x = -1e10
        for line in f.readlines():
            if line.startswith("#"):
                continue
            line = line.split()[3:]
            if line[0] == "position":
                line = line[3:]
                last_x = float(line[0])
                last_y = float(line[1])
                last_theta = float(line[2])

            elif line[0] == "laser":
                if last_x == -1e10:
                    continue
                tstamp = float(line[2])
                line = line[7:]
                # print(len(line), line[-1])
                # break

                laser_scan = LaserScan()
                laser_scan.header.frame_id = "base_link"
                laser_scan.angle_min = math.radians(-90)
                laser_scan.angle_max = math.radians(90)
                laser_scan.angle_increment = math.radians(1)
                laser_scan.range_min = 0.0
                laser_scan.range_max = 4.0
                num_readings = 181
                for i in range(num_readings):
                    laser_scan.ranges.append(float(line[2 * i]))

                trans = TransformStamped()
                trans.header.frame_id = "odom"
                trans.child_frame_id = "base_link"
                trans.transform.translation.x = last_x
                trans.transform.translation.y = last_y
                trans.transform.translation.z = 0
                
                quat = tf.transformations.quaternion_from_euler(0.0, 0.0, last_theta)
                trans.transform.rotation.x = quat[0]
                trans.transform.rotation.y = quat[1]
                trans.transform.rotation.z = quat[2]
                trans.transform.rotation.w = quat[3]

                trans.header.stamp = laser_scan.header.stamp = rospy.Time(tstamp)
                trans.header.seq = laser_scan.header.seq = seq
                
                tf_msg = TFMessage()
                tf_msg.transforms.append(trans)
                bag.write("tf", tf_msg, trans.header.stamp)
                bag.write("scan", laser_scan, trans.header.stamp)
                seq += 1

                last_x = -1e10

    bag.close()


if __name__ == "__main__":
    # convert_radish("run02.log", "usc-sal200-021120")
    # convert_stanford_gates("stanford-gates1.log", "stanford-gates1")
    convert_carmen("fr079.txt", "fr079")
    # convert_carmen_bspline("fr079.txt", "fr079-spline.log")
    # convert_carmen("mit-csail.clf", "mit-csail")
