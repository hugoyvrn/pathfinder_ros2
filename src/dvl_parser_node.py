#! /usr/bin/python3

import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import String
from rclpy.node import Node
import numpy as np

"""
TS : Year, month, day, hour, minute, second, hundredths of second (YY MM DD HH mm ss hh); Salinité; température; Depth of transducer face in meters; Vitesse du son; bit d'erreur
RA : Pression (kPa), distance au sol (dm, 4, un par beam)
BI : INSTRUMENT-REFERENCED VELOCITY DATA : X, Y, Z, Erreur, Statut       | TwistWithCovarianceStamped
BS : SHIP-REFERENCED VELOCITY DATA : transverse, longitudinal, normal, statut
BE : EARTH-REFERENCED VELOCITY DATA : Est, Nord, Upward, statut
BD : EARTH-REFERENCED DISTANCE DATA : Est, Nord, Upward, Range to bottom, Time since last good-velocity estimate in seconds
SA : Pitch roll heading (3 floats)
"""


class StringSubscriberNode(Node):
    def __init__(self):
        super().__init__('dvl_parser_node')
        self.subscription = self.create_subscription(
            String,
            'dvl/data',
            self.raw_data_callback,
            10)
        self.publisher = self.create_publisher(
            TwistWithCovarianceStamped,
            'dvl/vel',
            10)

    def raw_data_callback(self, msg):
        #print(msg.data)
        split_msg_data = msg.data.split("time_stamped")
        time_stamp = float(split_msg_data[0])
        for ligne in split_msg_data[1].split("\n"):
            parsed = self.string_parser(ligne)
            #print(parsed)
            if parsed != None:
                ligne_type = parsed[0]
                linear_x_vel = parsed[1]
                linear_y_vel = parsed[2]
                linear_z_vel = parsed[3]
                error_linear_vel = parsed[4]

                #print("izquygsv")
                twist_message = TwistWithCovarianceStamped()
                twist_message.header.stamp.sec = int(time_stamp)
                twist_message.header.stamp.nanosec = int((time_stamp % 1)*1e9)
                twist_message.header.frame_id = 'dvl'
                twist_message.twist.twist.linear.x = linear_x_vel/1000
                twist_message.twist.twist.linear.y = linear_y_vel/1000
                twist_message.twist.twist.linear.z = linear_z_vel/1000
                twist_message.twist.twist.angular.x = 0.0
                twist_message.twist.twist.angular.y = 0.0
                twist_message.twist.twist.angular.z = 0.0
                cov = np.diag([error_linear_vel, error_linear_vel,
                        error_linear_vel, -1., -1., -1.])
                twist_message.twist.covariance = list(cov.flatten())
                self.publisher.publish(twist_message)

    def string_parser(self,ligne):
        BI_ligne = ligne.replace("\n", "").replace("\r", "").split(",")

        if (BI_ligne[0] == ":BI"):
            BI_ligne = [BI_ligne[0]] + \
                [float(el) for el in BI_ligne[1:-1]] + [BI_ligne[-1]]
            if -32768.0 in BI_ligne[1:-1]:
                ligne_id, linear_x_vel, linear_y_vel, linear_z_vel, error_linear_vel, _ = BI_ligne
            #print(ligne_id, linear_x_vel, linear_y_vel, linear_z_vel, error_linear_vel)
            return [ligne_id, linear_x_vel, linear_y_vel, linear_z_vel, error_linear_vel]
        #print(BI_ligne)



def main(args=None):
    rclpy.init(args=args)

    string_subscriber_node = StringSubscriberNode()

    rclpy.spin(string_subscriber_node)

    string_subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()











import time
import argparse
from rosbag_creator import RosBagCreator
from geometry_msgs.msg import TwistWithCovarianceStamped
from geometry_msgs.msg import Quaternion
import numpy as np
from tools import Proj


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = np.cos(ai)
    si = np.sin(ai)
    cj = np.cos(aj)
    sj = np.sin(aj)
    ck = np.cos(ak)
    sk = np.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


def ros_quaternion_from_euler(ai, aj, ak):
    ros_quaternion = Quaternion()
    ros_quaternion.x, ros_quaternion.y, ros_quaternion.z, ros_quaternion.w = quaternion_from_euler(
        ai, aj, ak)
    return ros_quaternion


class DVL_parser(RosBagCreator):
    def __init__(self):
        parser = argparse.ArgumentParser(
            prog='dvl_log_to_rosbag',
            description='.oculus file parser to rosbag.')
        parser.add_argument('filename', type=str,
                            help='Path to a .oculus file to display')
        parser.add_argument('destination', type=str,
                            help='Path to the folder to save the rosbag.')
        parser.add_argument('--secondsoffset', type=float, default=0,
                            help="Set a time offset in seconds. Useful to handle time lignes to utc. A secondsoffset of 1 will write the rosbag 1s later that writen in the log file. Default to 0")
        parser.add_argument('-st', '--startingtime', type=float, default=0,
                            help='Nombre of seconds since the Epoch for the rosbag startingtime. No data timestamped before startingtime will be taken into acount. Default to None.')
        parser.add_argument('-et', '--endingtime',  type=float, default=0,
                            help='Nombre of seconds since the Epoch for the rosbag endingtime. No data timestamped after endingtime will be taken into acount. Default to None.')
        parser.add_argument('-ns', '--namespace', type=str, default='dvl',
                            help='Name space for all thetopics from the DVL. Default to "dvl".')
        parser.add_argument('-tn', '--topicname', type=str, default='vel',
                            help='Name of the topic. Default to "vel".')
        self.args = parser.parse_args()

        self.output_pass = self.args.destination + "/" + \
            self.args.filename.split("/")[-1][:-4]
        self.output_pass = self.output_pass.replace("//", "/")
        self.output_pass = self.output_pass.replace("/./", "/")
        super().__init__(self.output_pass)

        self.dt = (19.29-17.29)/10 #pas propre
        self.freq = 1*self.dt

    def readline(self):
        ligne = self.log.readline()
        if ligne != '\n':
            return ligne
        return self.readline()

    def read_file_header(self):
        print('[dvl_log_to_rosbag] Opening', self.args.filename)
        self.log = open(self.args.filename, 'r')
        # No header

    def init_topics(self):
        self.topicname_vel = self.args.namespace + "/" + self.args.topicname

        self.new_topic(name=self.topicname_vel,
                       type="geometry_msgs/msg/TwistWithCovarianceStamped")

    def create_ros_twist_msg(self, seconds, linear_x_vel, linear_y_vel, linear_z_vel, error_linear_vel):

        ros_msg = TwistWithCovarianceStamped()

        ros_msg.header.stamp.sec = int(seconds)
        ros_msg.header.stamp.nanosec = int((seconds % 1)*1e9)

        ros_msg.header.frame_id = 'map'
        ros_msg.twist.twist.linear.x = linear_x_vel
        ros_msg.twist.twist.linear.y = linear_y_vel
        ros_msg.twist.twist.linear.z = linear_z_vel
        ros_msg.twist.twist.angular.x = 0.0
        ros_msg.twist.twist.angular.y = 0.0
        ros_msg.twist.twist.angular.z = 0.0
        cov = np.diag([error_linear_vel, error_linear_vel,
                      error_linear_vel, -1., -1., -1.])
        ros_msg.twist.covariance = list(cov.flatten())

        return ros_msg

    def seconds_from_time_stamp(self, time_stamp, format="%Y%m%d%H%M%S"):
        seconds = time.mktime(time.strptime(
            "20"+time_stamp[:-2], format)) + float(time_stamp[-2:])*1e-2 - time.timezone
        seconds += self.args.secondsoffset
        return seconds

    def parse_next(self):
        try:
            # if True:
            time_ligne = self.readline()
            if time_ligne == '':
                print("End of the reading.")
                return False
            elif time_ligne == '\n':
                return self.parse_next()
            elif time_ligne.split(",")[0] != ":TS":
                # print(">>", time_ligne)
                return self.parse_next()
            TS_ligne = time_ligne.replace("\n", "").split(",")
            TS_ligne = TS_ligne[0:2] + [float(el) for el in TS_ligne[2:]]
            if not (TS_ligne[0] == ":TS" and len(TS_ligne) == 7):
                print("\nTS_ligne = ", TS_ligne, "\n")
                assert (TS_ligne[0] == ":TS")
                assert (len(TS_ligne) == 7)
            ligne_id, time_stamp, salinity, temperature, transducer_depth, sound_speed, erreur_bit = TS_ligne
            seconds = self.seconds_from_time_stamp(time_stamp)
            if self.args.startingtime and (seconds + self.dt < self.args.startingtime):
                # for t in range(2*7*int(self.freq*(self.args.startingtime-seconds))-1):
                #     self.log.readline()
                for t in range(2*7-1):
                    self.log.readline()
                print('again', end=' ')
                return self.parse_next()
            elif self.args.endingtime and (self.args.endingtime + self.dt < seconds):
                return False

            RA_ligne = self.readline().replace("\n", "").split(",")
            RA_ligne = [RA_ligne[0]] + \
                [float(el) for el in RA_ligne[1:]]
            if not (RA_ligne[0] == ":RA" and len(RA_ligne) == 6):
                print("\nRA_ligne = ", RA_ligne, "\n")
                assert (RA_ligne[0] == ":RA")
                assert (len(RA_ligne) == 6)
            ligne_id, _, _, _, _, _ = RA_ligne

            BI_ligne = self.readline().replace("\n", "").split(",")
            BI_ligne = [BI_ligne[0]] + \
                [float(el) for el in BI_ligne[1:-1]] + [BI_ligne[-1]]
            if not (BI_ligne[0] == ":BI" and len(BI_ligne) == 6):
                print("\nBI_ligne = ", BI_ligne, "\n")
                assert (BI_ligne[0] == ":BI")
                assert (len(BI_ligne) == 6)
            ligne_id, linear_x_vel, linear_y_vel, linear_z_vel, error_linear_vel, _ = BI_ligne

            BS_ligne = self.readline().replace("\n", "").split(",")
            BS_ligne = [BS_ligne[0]] + \
                [float(el) for el in BS_ligne[1:-1]] + [BS_ligne[-1]]
            if not (BS_ligne[0] == ":BS" and len(BS_ligne) == 5):
                print("\nBS_ligne = ", BS_ligne, "\n")
                assert (BS_ligne[0] == ":BS")
                assert (len(BS_ligne) == 5)
            ligne_id, _, _, _, _ = BS_ligne

            BE_ligne = self.readline().replace("\n", "").split(",")
            BE_ligne = [BE_ligne[0]] + \
                [float(el) for el in BE_ligne[1:-1]] + [BE_ligne[-1]]
            if not (BE_ligne[0] == ":BE" and len(BE_ligne) == 5):
                print("\nBE_ligne = ", BE_ligne, "\n")
                assert (BE_ligne[0] == ":BE")
                assert (len(BE_ligne) == 5)
            ligne_id, _, _, _, _ = BE_ligne

            BD_ligne = self.readline().replace("\n", "").split(",")
            BD_ligne = [BD_ligne[0]] + \
                [float(el) for el in BD_ligne[1:]]
            if not (BD_ligne[0] == ":BD" and len(BD_ligne) == 6):
                print("BD_ligne = ", BD_ligne)
                assert (BD_ligne[0] == ":BD")
                assert (len(BD_ligne) == 6)
            ligne_id, _, _, _, _, _ = BD_ligne

            SA_ligne = self.readline().replace("\n", "").split(",")
            SA_ligne = [SA_ligne[0]] + \
                [float(el) for el in SA_ligne[1:]]
            if not (SA_ligne[0] == ":SA" and len(SA_ligne) == 4):
                print("\nSA_ligne = ", SA_ligne, "\n")
                assert (SA_ligne[0] == ":SA")
                assert (len(SA_ligne) == 4)
            ligne_id, _, _, _ = SA_ligne

        except ValueError as e:
            print("\nErreur de fichier log ou fin")
            # assert(False) # to handle
            print("\n", e, "\n")
            return self.parse_next()
        except AssertionError as e:
            assert(self.log.readline()=='')
            return False
        # except :
        #     print("\nErreur de fichier log")
        #     return False

        twist_ros_msg = self.create_ros_twist_msg(
            seconds, linear_x_vel, linear_y_vel, linear_z_vel, error_linear_vel)

        self.publish(self.topicname_vel, twist_ros_msg, seconds*1e9)

        return True

    def parse(self):

        print("[dvl_log_to_rosbag] Parsing {} to {}. This may take few seconds.".format(
            self.args.filename, self.output_pass))

        self.read_file_header()
        self.init_topics()

        k = 0
        start_time = time.perf_counter()

        while self.parse_next():
            k += 1
            # print(k)
            # print("coucou\n")
            elapsed_time = time.perf_counter() - start_time
            print('[dvl_log_to_rosbag] {} raw data have been parsed in {:.2f} seconds.'.format(
                k, elapsed_time), end='\r')

        print("\n")
        self.log.close()
        print("[dvl_log_to_rosbag] File parsing is done")


if __name__ == '__main__':
    parseur = DVL_parser()
    parseur.parse()
