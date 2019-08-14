#!/usr/bin/env python

# map_updater.py
# this node publishes the drivable track center and track with around a circle given current position of a car
# updates at a certain frequency
# this file is to work with optimization based game-theoretic planner
# author: mingyuw@stanford.edu

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry


import numpy as np
from scipy.spatial import KDTree

class odom_state(object):
    def __init__(self):
        self.time = None
        self.x = None
        self.y = None
        self.yaw = None
        self.vx = None
        self.vy = None
        self.speed = None

    def update_vehicle_state(self, odom_msg):
        self.time = odom_msg.header.stamp.to_sec()
        self.x = odom_msg.pose.pose.position.x
        self.y = odom_msg.pose.pose.position.y
        ori_quat = (odom_msg.pose.pose.orientation.x,
                    odom_msg.pose.pose.orientation.y,
                    odom_msg.pose.pose.orientation.z,
                    odom_msg.pose.pose.orientation.w)
        ori_euler = tf.transformations.euler_from_quaternion(ori_quat)
        self.yaw = ori_euler[2]
        self.vx = odom_msg.twist.twist.linear.x
        self.vy = odom_msg.twist.twist.linear.y
        self.speed = np.sqrt(self.vx**2 + self.vy**2)
        # print("this is the current speed", self.speed)

    def get_position(self):
        return [self.x, self.y]

    def get_pose(self):
        return [self.x, self.y, self.yaw]

    def get_velocity(self):
        return [self.vs, self.vy]

    def get_speed(self):
        return self.speed


class MapUpdater:
    def __init__(self):
        rospy.init_node("map_updater", anonymous=True)

        # retrieve ros parameters
        self.steps = rospy.get_param("~steps")      # # of waypoints in the track
        self.distance = rospy.get_param("~distance")     # distance between two waypoints
        freq = rospy.get_param("~update_frequency")

        # state information
        self.stateReady = False
        self.state = odom_state()
        self.ado_stateReady = False
        self.ado_state = odom_state()

        # path information
        self.ego_track_info = Path()
        self.ado_track_info = Path()

        # subscribers, publishers
        rospy.Subscriber("/MSLcar0/ground_truth/odometry", Odometry, self.odom_cb)
        rospy.Subscriber("/MSLcar1/ground_truth/odometry", Odometry, self.ado_odom_cb)
        self.ego_track_pub = rospy.Publisher("/MSLcar0/mpc/ego_track_information", Path, queue_size=10)
        self.ado_track_pub = rospy.Publisher("/MSLcar0/mpc/ado_track_information", Path, queue_size=10)
        self.update_timer = rospy.Timer(rospy.Duration(1.0/freq), self.timer_cb)

        if True:
            self.entrance_1_tree = KDTree([[46.48933792114258, -6.036137580871582], [44.86451721191406, -6.060408115386963], [43.239620208740234, -6.084680557250977], [41.61479949951172, -6.108951091766357], [39.9899787902832, -6.1332221031188965], [38.36516189575195, -6.1574931144714355], [36.736148834228516, -6.183269500732422], [35.10575485229492, -6.214203357696533], [33.47547149658203, -6.250435829162598], [31.845312118530273, -6.291966438293457], [30.21529769897461, -6.338794708251953], [28.585447311401367, -6.390920639038086], [26.944692611694336, -6.448661804199219], [25.10228729248047, -6.6499223709106445], [23.302072525024414, -7.09058141708374], [21.575014114379883, -7.763057708740234], [19.950820922851562, -8.655783653259277], [18.457439422607422, -9.753400802612305], [17.120553970336914, -11.037027359008789], [16.007230758666992, -12.389640808105469], [15.024080276489258, -13.627730369567871], [14.029878616333008, -14.73604965209961], [12.915298461914062, -15.851956367492676], [11.810543060302734, -16.85011863708496], [10.631769180297852, -17.75967788696289], [9.386062622070312, -18.57516860961914], [8.080914497375488, -19.291685104370117], [6.724172115325928, -19.904922485351562], [5.323992729187012, -20.411190032958984], [3.888796091079712, -20.807445526123047], [2.427210569381714, -21.091306686401367], [0.9480249285697937, -21.261066436767578], [-0.5400566458702087, -21.31570053100586], [-2.0277085304260254, -21.254871368408203], [-3.5777547359466553, -21.086015701293945], [-5.186107158660889, -20.89655113220215], [-6.6851019859313965, -20.6679744720459], [-8.167065620422363, -20.347009658813477], [-9.626309394836426, -19.934892654418945], [-11.057233810424805, -19.43320655822754]])
            self.entrance_2_tree = KDTree([[-4.817941188812256, -39.5645751953125], [-4.931615829467773, -37.914188385009766], [-5.072919845581055, -36.26593780517578], [-5.2418131828308105, -34.62028884887695], [-5.438249588012695, -32.977691650390625], [-5.662173271179199, -31.338623046875], [-5.913521766662598, -29.703533172607422], [-6.2060627937316895, -28.0336856842041], [-6.633067607879639, -26.313678741455078], [-7.203592300415039, -24.63580322265625], [-7.913572788238525, -23.01201629638672], [-8.757951736450195, -21.453880310058594], [-9.730714797973633, -19.972497940063477], [-10.824929237365723, -18.578420639038086], [-12.032805442810059, -17.281579971313477], [-13.345735549926758, -16.091211318969727], [-14.59863567352295, -15.06821060180664], [-15.700803756713867, -14.067193984985352], [-16.814422607421875, -12.950329780578613], [-17.805749893188477, -11.83944034576416], [-18.708024978637695, -10.65507984161377], [-19.515817642211914, -9.40436840057373], [-20.224273681640625, -8.094826698303223], [-20.829132080078125, -6.734328746795654], [-21.32675552368164, -5.331054210662842], [-21.714153289794922, -3.8934409618377686], [-21.988996505737305, -2.430133104324341], [-22.149629592895508, -0.9499288201332092], [-22.19508934020996, 0.5384615063667297], [-22.125085830688477, 2.025710344314575], [-21.940059661865234, 3.503063440322876], [-21.641117095947266, 4.961638927459717], [-21.230058670043945, 6.392666339874268], [-20.7093563079834, 7.787541389465332], [-20.082138061523438, 9.13787841796875], [-19.3521785736084, 10.435554504394531], [-18.52386474609375, 11.672771453857422], [-17.602176666259766, 12.84208869934082], [-16.59265899658203, 13.936474800109863], [-15.400768280029297, 15.188582420349121]])
            self.entrance_3_tree = KDTree([[-34.814674377441406, -0.47130560874938965], [-33.29544448852539, -0.15828955173492432], [-31.790752410888672, 0.21845054626464844], [-30.303266525268555, 0.6582458019256592], [-28.937562942504883, 1.1595783233642578], [-27.651111602783203, 1.7960319519042969], [-26.347867965698242, 2.602447032928467], [-25.255699157714844, 3.4120240211486816], [-24.312442779541016, 4.375119209289551], [-23.48154067993164, 5.530763626098633], [-22.652679443359375, 6.847446441650391], [-21.87640380859375, 8.195799827575684], [-21.102096557617188, 9.63101863861084], [-20.19000244140625, 11.136578559875488], [-19.16399574279785, 12.566939353942871], [-18.030244827270508, 13.913501739501953], [-16.862815856933594, 15.119731903076172], [-15.867419242858887, 16.229799270629883], [-14.959613800048828, 17.41257667541504], [-14.14469051361084, 18.661163330078125], [-13.427399635314941, 19.96828269958496], [-12.81192398071289, 21.326316833496094], [-12.301850318908691, 22.72734832763672], [-11.900151252746582, 24.1632137298584], [-11.59899616241455, 25.65958595275879], [-11.310633659362793, 27.2587947845459], [-11.031770706176758, 28.83707618713379], [-10.788163185119629, 30.403366088867188], [-10.580278396606445, 31.974796295166016], [-10.408222198486328, 33.55055236816406], [-10.272087097167969, 35.12981414794922], [-10.171940803527832, 36.711769104003906], [-10.107836723327637, 38.29559326171875], [-10.079802513122559, 39.880516052246094], [-10.069519996643066, 41.5054817199707], [-10.05923843383789, 43.13045120239258], [-10.048955917358398, 44.75541687011719], [-10.038674354553223, 46.38038635253906], [-10.02839183807373, 48.00535202026367], [-10.018110275268555, 49.63032150268555]])
            self.entrance_4_tree = KDTree([[3.2099719047546387, 39.47578811645508], [3.321388006210327, 37.78154754638672], [3.503384828567505, 36.09342575073242], [3.755645751953125, 34.414371490478516], [4.077731609344482, 32.74729537963867], [4.469081878662109, 31.095111846923828], [4.925609588623047, 29.469890594482422], [5.389627456665039, 27.912548065185547], [5.935444355010986, 26.26019859313965], [6.607458114624023, 24.638778686523438], [7.398468971252441, 23.071962356567383], [8.304075241088867, 21.56847381591797], [9.319238662719727, 20.136672973632812], [10.438308715820312, 18.784530639648438], [11.655058860778809, 17.519569396972656], [12.962719917297363, 16.34882926940918], [14.146451950073242, 15.350162506103516], [15.250914573669434, 14.278253555297852], [16.309186935424805, 13.17756462097168], [17.25775146484375, 12.029940605163574], [18.114538192749023, 10.812270164489746], [19.139684677124023, 9.346755981445312], [20.363069534301758, 7.976857662200928], [21.743579864501953, 6.765460014343262], [23.26082992553711, 5.730450630187988], [24.89241600036621, 4.887112617492676], [26.61424446105957, 4.247899532318115], [28.400896072387695, 3.822249412536621], [30.225982666015625, 3.616448163986206], [31.93506622314453, 3.5933148860931396], [33.56005859375, 3.588041067123413], [35.18888854980469, 3.5839576721191406], [36.819576263427734, 3.5848898887634277], [38.45024871826172, 3.5911219120025635], [40.08089828491211, 3.6026535034179688], [41.71149444580078, 3.6194849014282227], [43.34157943725586, 3.641594171524048], [44.966392517089844, 3.665865182876587], [46.591209411621094, 3.690136194229126], [48.216026306152344, 3.714406967163086]])

    def odom_cb(self, msg):
        self.state.update_vehicle_state(msg)
        if not self.stateReady:
            self.stateReady = True


    def ado_odom_cb(self, msg):
        self.ado_state.update_vehicle_state(msg)
        if not self.ado_stateReady:
            self.ado_stateReady = True

    def get_path_from_position(self, position_x, position_y):

        track_center = np.zeros((2, self.steps))
        track_width = 3.5
        if abs(position_x) < 25 and abs(position_y) < 25:
            pos_the = np.arctan2(position_y, position_x)
            the = np.linspace(pos_the-0.5, pos_the - 0.5 + np.pi, self.steps)

            radius = 21.75
            for i in range(self.steps):
                track_center[:,i] = [radius*np.cos(the[i]), radius*np.sin(the[i])]

        elif position_x > 24.9:
            _, idx = self.entrance_1_tree.query([position_x, position_y])
            track_center = self.entrance_1_tree.data[idx - 2: idx + 18, :].T.copy()
            track_center[1,:] = -track_center[1,:]
        elif position_x < -24.9:
            _, idx = self.entrance_3_tree.query([position_x, position_y])
            track_center = self.entrance_3_tree.data[idx - 2: idx + 18, :].T.copy()
            track_center[1,:] = -track_center[1,:]
        elif position_y < -24.9:
            _, idx = self.entrance_4_tree.query([position_x, position_y])
            track_center = self.entrance_4_tree.data[idx - 2: idx + 18, :].T.copy()
            track_center[1,:] = -track_center[1,:]
        else:
            _, idx = self.entrance_2_tree.query([position_x, position_y])
            track_center = self.entrance_2_tree.data[idx - 2: idx + 18, :].T.copy()
            track_center[1,:] = -track_center[1,:]
        return track_center, track_width

    def update_ego_track(self):
        if not self.stateReady:
            return
        ego_pos = self.state.get_position()
        track_c, track_w = self.get_path_from_position(ego_pos[0], ego_pos[1])
        self.ego_track_info.header.stamp = rospy.Time.now()
        self.ego_track_info.header.frame_id = "map"
        self.ego_track_info.poses = []
        for i in range(self.steps):
            pose_s = PoseStamped()
            pose_s.header = self.ego_track_info.header
            pose_s.pose.position.x = track_c[0,i]
            pose_s.pose.position.y = track_c[1,i]
            self.ego_track_info.poses.append(pose_s)
        # append the track width info as the last element of path.poses
        track_w_pose = PoseStamped()
        track_w_pose.header = self.ego_track_info.header
        track_w_pose.pose.position.x = track_w
        self.ego_track_info.poses.append(track_w_pose)


    def update_ado_track(self):
        ado_pos = self.ado_state.get_position()
        track_c, track_w = self.get_path_from_position(ado_pos[0], ado_pos[1])
        self.ado_track_info.header.stamp = rospy.Time.now()
        self.ado_track_info.header.frame_id = "map"
        self.ado_track_info.poses = []
        for i in range(self.steps):
            pose_s = PoseStamped()
            pose_s.header = self.ado_track_info.header
            pose_s.pose.position.x = track_c[0,i]
            pose_s.pose.position.y = track_c[1,i]
            self.ado_track_info.poses.append(pose_s)
        # append the track width info as the last element of path.poses
        track_w_pose = PoseStamped()
        track_w_pose.header = self.ado_track_info.header
        track_w_pose.pose.position.x = track_w
        self.ado_track_info.poses.append(track_w_pose)


    def timer_cb(self, event):

        # lazy update of track information
        if self.stateReady and self.ado_stateReady:
            self.update_ego_track()
            self.update_ado_track()

            self.ego_track_pub.publish(self.ego_track_info)
            self.ado_track_pub.publish(self.ado_track_info)

if __name__ == '__main__':
    try:
        supervisor = MapUpdater()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
