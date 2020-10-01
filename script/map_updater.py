#!/usr/bin/env python

# map_updater.py
# this node publishes the drivable track center and track with around a circle
# given current position of a car
# updates at a certain frequency
# this file is to work with optimization based game-theoretic planner
# author: mingyuw@stanford.edu

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path, Odometry


import numpy as np
from scipy.spatial import KDTree
import random

import carla
from carla_circle.srv import GetAvailablePath


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
        ori_quat = (
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w
        )
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
        self.steps = rospy.get_param("~steps")  # Num of waypoints in the track
        self.distance = rospy.get_param("~distance")  # dist btw 2 waypoints
        freq = rospy.get_param("~update_frequency")
        exit_time = rospy.get_param("~exit_time")
        self.max_speed = rospy.get_param("~max_speed")
        self.opp_speed = rospy.get_param("~opp_speed")
        self.plan_horizon = rospy.get_param("~plan_horizon")

        # state information
        self.stateReady = False
        self.state = odom_state()
        self.ado_stateReady = False
        self.ado_state = odom_state()

        self.current_stage = 1  # 1 -- start and circle     2 -- existing

        # path information
        self.ego_track_info = Path()
        self.ado_track_info = Path()

        # service proxy to get a path update from carla world
        rospy.wait_for_service("get_path", 8.0)
        self.get_path_handle = rospy.ServiceProxy('get_path', GetAvailablePath)

        # Subscribers for ego and opponent vehicle odometry
        rospy.Subscriber(
            "MSLcar0/ground_truth/odometry",
            Odometry,
            self.odom_cb
        )
        rospy.Subscriber(
            "MSLcar1/ground_truth/odometry",
            Odometry,
            self.ado_odom_cb
        )

        # Publishers for track information of ego and opponent
        self.ego_track_pub = rospy.Publisher(
            "MSLcar0/mpc/ego_track_information",
            Path,
            queue_size=10
        )
        self.ado_track_pub = rospy.Publisher(
            "MSLcar0/mpc/ado_track_information",
            Path,
            queue_size=10
        )

        # Class timers
        self.update_timer = rospy.Timer(
            rospy.Duration(1.0/freq),
            self.timer_cb
        )
        self.exit_timer = rospy.Timer(
            rospy.Duration(exit_time),
            self.exit_cb
        )

        if True:
            self.entrance_1_tree = KDTree([[57.08933792114258, 6.036137580871582], [53.48933792114258, 6.036137580871582], [50.08933792114258, 6.036137580871582], [46.48933792114258, 6.036137580871582], [43.239620208740234, 6.084680557250977], [39.9899787902832, 6.1332221031188965], [36.736148834228516, 6.183269500732422], [33.47547149658203, 6.250435829162598], [30.21529769897461, 6.338794708251953], [26.944692611694336, 6.448661804199219], [23.302072525024414, 7.09058141708374], [19.950820922851562, 8.655783653259277], [17.120553970336914, 11.037027359008789], [15.024080276489258, 13.627730369567871], [12.915298461914062, 15.851956367492676], [10.631769180297852, 17.75967788696289], [8.080914497375488, 19.291685104370117], [5.323992729187012, 20.411190032958984], [2.427210569381714, 21.091306686401367], [-0.5400566458702087, 21.31570053100586], [-3.506174325942993, 21.07895851135254], [-6.400100231170654, 20.386791229248047], [-9.152338981628418, 19.25581932067871], [-11.696793556213379, 17.713207244873047], [-13.972362518310547, 15.795997619628906], [-16.07497215270996, 13.566105842590332], [-17.781204223632812, 11.12834358215332], [-19.09031105041504, 8.456239700317383], [-19.970857620239258, 5.613962173461914], [-20.40169906616211, 2.669767379760742], [-20.37247085571289, -0.3058292865753174], [-19.883880615234375, -3.2409920692443848], [-18.947669982910156, -6.06542444229126], [-17.58632469177246, -8.711297035217285], [-15.832539558410645, -11.115072250366211], [-13.701011657714844, -13.318000793457031], [-11.388221740722656, -15.190141677856445], [-8.813955307006836, -16.682472229003906], [-6.040032863616943, -17.7591609954834], [-3.133068084716797, -18.394350051879883], [-0.16268262267112732, -18.57278060913086], [2.799415111541748, -18.290143966674805], [5.682278156280518, -17.55325698852539]])
            self.entrance_2_tree = KDTree([[-3.745779037475586, 80.46041107177734], [-3.825951337814331, 77.2114028930664], [-3.906123638153076, 73.96238708496094], [-3.9862961769104004, 70.71337890625], [-4.066468238830566, 67.46437072753906], [-4.146640777587891, 64.2153549194336], [-4.226813316345215, 60.96635055541992], [-4.306985855102539, 57.71733856201172], [-4.387157917022705, 54.468326568603516], [-4.467329978942871, 51.21931838989258], [-4.547502517700195, 47.970306396484375], [-4.6276750564575195, 44.72129440307617], [-4.720769882202148, 41.4802360534668], [-4.911633491516113, 38.17726516723633], [-5.213029861450195, 34.88254928588867], [-5.6246209144592285, 31.599773406982422], [-6.151449203491211, 28.31108283996582], [-7.103138446807861, 24.90007781982422], [-8.61451530456543, 21.6975154876709], [-10.642590522766113, 18.794485092163086], [-13.129678726196289, 16.273561477661133], [-15.530281066894531, 14.232523918151855], [-17.65346908569336, 12.021712303161621], [-19.39352798461914, 9.607978820800781], [-20.73976707458496, 6.954388618469238], [-21.659860610961914, 4.124665260314941], [-22.131715774536133, 1.1867645978927612], [-22.14398193359375, -1.7889500856399536], [-21.696361541748047, -4.730640888214111], [-20.799625396728516, -7.56785249710083], [-19.47530746459961, -10.232450485229492], [-17.755207061767578, -12.660446166992188], [-15.644153594970703, -14.882573127746582], [-13.357693672180176, -16.786781311035156], [-10.804485321044922, -18.314863204956055], [-8.045845031738281, -19.430126190185547], [-5.1480207443237305, -20.10578727722168], [-2.1804118156433105, -20.32561683654785], [0.7853388786315918, -20.08431053161621], [3.678196907043457, -19.387691497802734], [6.428691864013672, -18.252492904663086], [8.970771789550781, -16.705970764160156], [11.243388175964355, -14.785260200500488], [13.34371280670166, -12.5532808303833], [15.046192169189453, -10.112898826599121], [16.3511905670166, -7.438783645629883], [17.22736358642578, -4.595155239105225], [17.653676986694336, -1.6503015756607056], [17.619873046875, 1.3252460956573486], [17.126766204833984, 4.259654521942139]])
            self.entrance_3_tree = KDTree([[-46.47760009765625, 1.1062910556793213], [-43.22764587402344, 1.0895800590515137], [-40.03771209716797, 1.0517759323120117], [-36.94639205932617, 0.7996401190757751], [-33.88726043701172, 0.2881685495376587], [-30.882049560546875, -0.4790079593658447], [-28.145339965820312, -1.5309288501739502], [-25.66337776184082, -3.078059673309326], [-23.816064834594727, -5.030394077301025], [-22.173274993896484, -7.665506362915039], [-20.560134887695312, -10.556978225708008], [-18.48548126220703, -13.398045539855957], [-16.100404739379883, -15.86613655090332], [-13.365116119384766, -18.078386306762695], [-10.320878028869629, -19.84147071838379], [-7.040793418884277, -21.113046646118164], [-3.603635311126709, -21.86258316040039], [-0.09172112494707108, -22.072071075439453], [3.4101669788360596, -21.736454010009766], [6.818153381347656, -20.86382484436035], [10.050397872924805, -19.475139617919922], [13.029279708862305, -17.603742599487305], [15.673657417297363, -15.305257797241211], [18.00530433654785, -12.768614768981934], [19.976469039916992, -9.854790687561035], [21.473777770996094, -6.671408653259277], [22.461273193359375, -3.2949142456054688], [22.915241241455078, 0.1936054229736328], [22.824758529663086, 3.7105984687805176], [22.192007064819336, 7.171159744262695], [21.03219223022461, 10.49240779876709], [19.373170852661133, 13.594585418701172], [17.25478172302246, 16.403194427490234], [15.028800964355469, 18.78847312927246], [13.1324462890625, 21.169652938842773], [11.526254653930664, 23.75544548034668], [10.232078552246094, 26.510671615600586], [9.267524719238281, 29.397851943969727], [8.579553604125977, 32.44445037841797], [8.066237449645996, 35.552730560302734]])
            self.entrance_4_tree = KDTree([[3.2012617588043213, -46.50347137451172], [3.1806976795196533, -43.253536224365234], [3.18855619430542, -40.04135513305664], [3.4348978996276855, -36.65523910522461], [3.962650775909424, -33.30144119262695], [4.7681403160095215, -30.003311157226562], [5.738562107086182, -26.811220169067383], [7.121852397918701, -23.587589263916016], [8.969036102294922, -20.60544204711914], [11.239055633544922, -17.93106460571289], [13.774421691894531, -15.6787691116333], [15.973599433898926, -13.54331111907959], [17.839487075805664, -11.225472450256348], [19.324861526489258, -8.647185325622559], [20.394054412841797, -5.870365619659424], [21.02138900756836, -2.9616963863372803], [21.19179916381836, 0.009160074405372143], [20.901165008544922, 2.9704835414886475], [20.156496047973633, 5.851346492767334], [18.975671768188477, 8.582566261291504], [17.38705062866211, 11.098554611206055], [15.392755508422852, 13.397238731384277], [13.133936882019043, 15.926066398620605], [11.13284683227539, 18.73921775817871], [9.472932815551758, 21.7662410736084], [8.176774978637695, 24.965953826904297], [7.262565612792969, 28.272489547729492], [6.616960048675537, 31.55841827392578], [6.158554553985596, 34.87564468383789], [5.888822555541992, 38.21351623535156], [5.808630466461182, 41.56130599975586], [5.877533435821533, 44.84600830078125], [5.957705497741699, 48.09502029418945], [6.037878036499023, 51.34402847290039], [6.118050575256348, 54.593040466308594], [6.198222637176514, 57.8420524597168], [6.278395175933838, 61.091064453125], [6.358567237854004, 64.34007263183594], [6.438739776611328, 67.5890884399414], [6.518912315368652, 70.83809661865234]])
            self.exit_tree = KDTree([[-23.822586059570312, 3.7314443588256836], [-23.916318893432617, 0.21453630924224854], [-23.4655704498291, -3.2744016647338867], [-22.481191635131836, -6.6518049240112305], [-20.98682403564453, -9.836568832397461], [-19.018348693847656, -12.75221061706543], [-16.687334060668945, -15.291580200195312], [-14.046103477478027, -17.59124183654785], [-11.068950653076172, -19.465389251708984], [-7.837990760803223, -20.857059478759766], [-4.430810451507568, -21.73283576965332], [-0.9292333722114563, -22.071685791015625], [2.5828726291656494, -21.865440368652344], [6.020720958709717, -21.119077682495117], [9.301977157592773, -19.85053062438965], [12.347843170166016, -18.09025764465332], [15.085172653198242, -15.880531311035156], [17.471839904785156, -13.413958549499512], [19.528499603271484, -10.611713409423828], [21.343486785888672, -8.456718444824219], [23.623973846435547, -6.8021650314331055], [26.23576545715332, -5.745419979095459], [29.025171279907227, -5.348667621612549], [32.2463493347168, -5.336643695831299], [35.48533630371094, -5.335991382598877], [38.72388458251953, -5.356383800506592], [41.96418762207031, -5.3975510597229], [45.21382141113281, -5.44609260559082], [48.46345901489258, -5.49463415145874], [51.713096618652344, -5.543176174163818], [54.96273422241211, -5.591717720031738], [58.212371826171875, -5.640259742736816], [61.462005615234375, -5.688801288604736], [64.71165466308594, -5.737342834472656], [67.96128845214844, -5.785884857177734], [71.21092987060547, -5.834426403045654], [74.46056365966797, -5.882968425750732], [77.710205078125, -5.931509971618652], [80.95984649658203, -5.980051517486572], [84.20948028564453, -6.02859354019165]])

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
        track_width = 5.0

        if abs(position_x) < 25 and abs(position_y) < 25:
            radius = 21.75
            pos_the = np.arctan2(position_y, position_x)
            the_start = pos_the - 0.2
            angle = 2.0*self.max_speed*self.plan_horizon/radius
            the_end = the_start + angle
            the = np.linspace(the_start, the_end, self.steps)

            for i in range(self.steps):
                track_center[:, i] = [
                    radius*np.cos(the[i])-0.5, radius*np.sin(the[i])-0.5
                ]

        elif position_x > 24.9:
            _, idx = self.entrance_1_tree.query([position_x, position_y])
            if idx < 2:
                track_center = self.entrance_1_tree.data[idx: idx + 20, :].T.copy()
            else:
                track_center = self.entrance_1_tree.data[idx - 2: idx + 18, :].T.copy()
        elif position_x < -24.9:
            _, idx = self.entrance_3_tree.query([position_x, position_y])
            if idx < 2:
                track_center = self.entrance_3_tree.data[idx: idx + 20, :].T.copy()
            else:
                track_center = self.entrance_3_tree.data[idx - 2: idx + 18, :].T.copy()
        elif position_y < -24.9:
            _, idx = self.entrance_4_tree.query([position_x, position_y])
            if idx < 2:
                track_center = self.entrance_4_tree.data[idx: idx + 20, :].T.copy()
            else:
                track_center = self.entrance_4_tree.data[idx - 2: idx + 18, :].T.copy()
        else:
            _, idx = self.entrance_2_tree.query([position_x, position_y])
            if idx < 2:
                track_center = self.entrance_2_tree.data[idx: idx + 20, :].T.copy()
            else:
                track_center = self.entrance_2_tree.data[idx - 2: idx + 18, :].T.copy()
        return track_center, track_width

    def update_ego_track(self):
        ego_pos = self.state.get_position()
        track_c, track_w = self.get_path_from_position(ego_pos[0], ego_pos[1])
        self.ego_track_info.header.stamp = rospy.Time.now()
        self.ego_track_info.header.frame_id = "map"
        self.ego_track_info.poses = []

        for i in range(self.steps):
            pose_s = PoseStamped()
            pose_s.header = self.ego_track_info.header
            pose_s.pose.position.x = track_c[0, i]
            pose_s.pose.position.y = track_c[1, i]
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
            pose_s.pose.position.x = track_c[0, i]
            pose_s.pose.position.y = track_c[1, i]
            self.ado_track_info.poses.append(pose_s)

        # append the track width info as the last element of path.poses
        track_w_pose = PoseStamped()
        track_w_pose.header = self.ado_track_info.header
        track_w_pose.pose.position.x = track_w
        self.ado_track_info.poses.append(track_w_pose)

    def exit_cb(self, event):
        # change stage from stay/enter the circle to exit the circle
        if self.current_stage == 1:
            self.current_stage = 2
        print(" SWITCHED TO EXIT MODE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

    def update_ego_track_exit(self):
        # print(" UPDATE EXIT TRACK !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        ego_pos = self.state.get_position()
        track_c, track_w = self.get_exit_path(ego_pos[0], ego_pos[1])
        self.ego_track_info.header.stamp = rospy.Time.now()
        self.ego_track_info.header.frame_id = "map"
        self.ego_track_info.poses = []

        for i in range(self.steps):
            pose_s = PoseStamped()
            pose_s.header = self.ego_track_info.header
            pose_s.pose.position.x = track_c[0, i]
            pose_s.pose.position.y = track_c[1, i]
            self.ego_track_info.poses.append(pose_s)

        # append the track width info as the last element of path.poses
        track_w_pose = PoseStamped()
        track_w_pose.header = self.ego_track_info.header
        track_w_pose.pose.position.x = track_w
        self.ego_track_info.poses.append(track_w_pose)

    def get_exit_path(self, position_x, position_y):
        track_center = np.zeros((2, self.steps))
        track_width = 5.0
        pos_the = np.arctan2(position_y, position_x)

        if pos_the > 0:    # if the car is currently in the other half circle
            the = np.linspace(pos_the-0.5, pos_the - 0.5 + np.pi, self.steps)
            radius = 21.75

            for i in range(self.steps):
                track_center[:, i] = [
                    radius*np.cos(the[i]), radius*np.sin(the[i])
                ]
        else:  # if the car is in existing half circle
            if position_x < 30:  # still use our stored path information
                _, idx = self.exit_tree.query([position_x, position_y])

                if idx < 2:
                    track_center = self.exit_tree.data[idx: idx + 20, :].T.copy()
                else:
                    track_center = self.exit_tree.data[idx - 2: idx + 18, :].T.copy()
            else:
                current_pose = Pose()
                current_pose.position.x = position_x
                current_pose.position.y = position_y

                try:
                    track_width = 3.2
                    track_center = np.zeros((2, self.steps))
                    path_list_resp = self.get_path_handle(current_pose)
                    path = path_list_resp.paths.paths[0]
                    # print(" this is our beloved path", path)

                    for i in range(self.steps - 2):
                        track_center[0,i+2] = path.poses[i].pose.position.x
                        track_center[1,i+2] = path.poses[i].pose.position.y

                    track_center[0, 1] = 2*track_center[0, 2] - track_center[0, 3]
                    track_center[1, 1] = 2*track_center[1, 2] - track_center[1, 3]
                    track_center[0, 0] = 2*track_center[0, 1] - track_center[0, 2]
                    track_center[1, 0] = 2*track_center[1, 1] - track_center[1, 2]
                except rospy.ServiceException, e:
                    print "Service call failed: %s" % e
                # print(" track center ", track_center)

        return track_center, track_width

    def timer_cb(self, event):

        # lazy update of track information
        if self.stateReady and self.ado_stateReady:
            if self.current_stage == 1:
                # entering the circle or staying in the circle
                self.update_ego_track()
            else:
                # ego car needs to leave circle
                self.update_ego_track_exit()

            self.update_ado_track()

            self.ego_track_pub.publish(self.ego_track_info)
            self.ado_track_pub.publish(self.ado_track_info)


if __name__ == '__main__':
    try:
        supervisor = MapUpdater()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
