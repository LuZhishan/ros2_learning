#encoding=utf-8
# from platform import node
# from typing_extensions import Self
import this
import cv2
import numpy as np
import rclpy
import cv_bridge
import message_filters
import sensor_msgs.msg

from message_filters import ApproximateTimeSynchronizer
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
# from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs_py import point_cloud2 as pc2
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

import math
import sys
import os
import pyquaternion
import yaml

bridge = cv_bridge.CvBridge()

###################### for finetune ###########################
yaw = 0.0 # in deg
pitch = 0.0
roll = 0.0

x__ = 0.0
y__ = 0.0
z__ = 0.0

T_finetune = np.matrix([[1.0,0,0,0],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]])

camid=0

def get_T_finetune():
    global T_finetune
    return T_finetune

def deg2rad_(yaw,pitch,roll):
    return yaw*3.14159/180,pitch*3.14159/180,roll*3.14159/180

def euler_to_quaternion(yaw, pitch, roll):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return (qx, qy, qz, qw)

def refresh_T_finetune():
    global yaw,pitch,roll,x__,y__,z__,T_finetune
    y_,p_,r_ = deg2rad_(yaw,pitch,roll)
    x,y,z,w = euler_to_quaternion(y_,p_,r_)
    
    q = pyquaternion.Quaternion(w,x,y,z)
    T_finetune = q.transformation_matrix
    T_finetune[0,3] = x__
    T_finetune[1,3] = y__
    T_finetune[2,3] = z__
    print ('T_finetune:',T_finetune)

def f_(x):
    print (x)
    return float(x)

camNewK = None 
T_cam_lidar = None
width = 1440
height = 1080

# 待验证摄像头的内参和外参
def set_in_ext_param(camid):
    global camNewK,T_cam_lidar,image_topic,width,height
    if camid == 0:
        # 红绿灯摄像头
        camNewK = np.array([[2530.710693,0.000000,961.943302],
        [0.000000,2574.185547,795.249226],
        [0.000000,0.000000,1.000000]])

        #手眼微调
        T_cam_lidar = np.matrix([[-2.60082765e-01, -9.65585592e-01,  4.51182426e-04,  5.39967270e-01],
                [-2.97826712e-02,  7.55500508e-03, -9.99528012e-01,  7.16734900e-02],
                [ 9.65127011e-01, -2.59974024e-01, -3.07227046e-02,  5.09990660e-01],
                [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])


        # T_cam_lidar = np.matrix([[ -0.301906,  -0.953324, 0.00499018,    0.53996727],
        #     [-0.0468478, 0.00960762,  -0.998856,    0.07167349],
        #     [ 0.952186,  -0.301795, -0.0475618,    0.50999066],
        #     [ 0.        ,  0.        ,  0.        ,  1.        ]])
    elif camid == 1:
        # 前摄像头  tiscamera
        image_topic = '/image_raw'
        width = 1440
        height = 1080
        camNewK = np.array([[401.5881,0,724.75822],
        [0.     , 506.92841, 489.93506],
        [0.000000,0.000000,1.000000]])

        #手眼微调
        T_cam_lidar = np.matrix([[ 0.01666841, -0.99983836, -0.00673865, -0.19164723],
                [-0.16942405,  0.00381777, -0.98553585, -0.27016748],
                [ 0.98540228,  0.01756901, -0.16933303, -0.39299324],
                [ 0.,         0.,          0.,         1.        ]])

        # T_cam_lidar = np.matrix([[ 4.2835397166434627e-02,  -9.9905941731977932e-01, -6.7386507742220438e-03, -1.9164722716316049e-01],
        #     [-1.6946592673379540e-01, -6.1854012869663776e-04, -9.8553585276455857e-01, -2.7016748426325321e-01],
        #     [ 9.8460470668479438e-01,   4.3357791373319646e-02, -1.6933302543029818e-01, -3.9299324217351017e-01],
        #     [ 0.        ,  0.        ,  0.        ,  1.        ]])
        # 待验证摄像头的内参和外参

        #20220215 lishui
        T_cam_lidar=np.matrix([[ 0.06053493, -0.99815967, -0.0035754,  -0.19164723],
                           [-0.11761821, -0.00357603, -0.99305245, -0.27016748],
                           [ 0.99121213,  0.0605349,  -0.11761823, -0.39299324],
                           [ 0.,          0.,          0.,          1.        ]])

#         T_final=[[ 3.85875294e-02 -9.99236807e-01 -6.06612922e-03 -1.91647230e-01]
#  [-1.52241248e-01  1.20898424e-04 -9.88343353e-01 -2.70167480e-01]
#  [ 9.87589795e-01  3.90612484e-02 -1.52120395e-01 -3.92993240e-01]
#  [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]


    elif camid == 2:
        # 左摄像头
        image_topic = '/left_camera0'

        camNewK = np.array([[507,             0.0,            603],
        [0.0,             507,            484],
        [0.000000,0.000000,1.000000]])

        # 手眼微调
        T_cam_lidar = np.matrix([[ 0.97591167, -0.21734532, -0.0188862,   0.0734516 ],
                                [-0.0712218,  -0.23557441, -0.969243,   -0.186494  ],
                                [ 0.20621125,  0.94724091, -0.245379,   -0.479577  ],
                                [ 0.,          0.,          0.,          1.        ]])

        # T_cam_lidar = np.matrix([[ 0.98736,  -0.157362, -0.0188862,  0.0734516],
        #     [-0.0567075,  -0.239483,  -0.969243,  -0.186494],
        #     [ 0.147999,  0.958063,  -0.245379,  -0.479577],
        #     [ 0.        ,  0.        ,  0.        ,  1.        ]])
    elif camid == 3:
        # 右摄像头
        image_topic = '/right_camera0'

        camNewK = np.array([[4.0850945999999999e+02,             0.0,            8.0810414500000002e+02],
        [0.0,                    5.1616387899999995e+02,     5.4022584500000005e+02],
        [0.000000,0.000000,1.000000]])

        #手眼微调
        T_cam_lidar = np.matrix([[-0.99315555, -0.02916338, -0.11310115,  0.204192  ],
                [ 0.0911309,   0.41220609, -0.90652196,  0.0364233 ],
                [ 0.0730582,  -0.91062359, -0.40672663, -0.22989   ],
                [ 0.,          0.,          0.,          1.        ]])


        # T_cam_lidar = np.matrix([[ -0.993015, 0.0306502, -0.113939,  0.204192],
        #     [0.115912,  0.433824, -0.893511, 0.0364233],
        #     [0.0220432, -0.900476, -0.434346,  -0.22989],
        #     [ 0.        ,  0.        ,  0.        ,  1.        ]])

    else:
        print('unsupport cam,camid={}'.format(camid))

    print('T_cam_lidar={}'.format(T_cam_lidar))


def filter_xyz(p):
    global camid
    # for traffic_light camera
    if camid == 0:
        x_limit = (1,5)
        y_limit = (-1,2)    
        z_limit = (0,5)
    if camid == 1:
        # for front camera
        x_limit = (5,50)
        y_limit = (-1,3)    
        z_limit = (-1,3)
    if camid == 2:
        # for left camera
        y_limit = (0,50)
        x_limit = (-3,3)
        z_limit = (-2,3)
    if camid == 3:
        # for right camera
        y_limit = (-50,0)
        x_limit = (-6,16)
        z_limit = (-2,3)
    if x_limit[0] < p[0] and x_limit[1] > p[0] \
        and y_limit[0] < p[1] and y_limit[1] > p[1] \
        and z_limit[0] < p[2] and z_limit[1] > p[2]:
        print('point:{}'.format(p))
        return True
    # print(camid)
    # print(x_limit[0],y_limit,z_limit)
    # print('bbbbbbbbbbbbbbbb')
    return False

    # return True

def __callback_func__(img_msg,lidar_msg):
    print("get in callback")
    global T_finetune,yaw,pitch,roll,x__,y__,z__

    print('sync!')
    try:
        global bridge
        print('sync!...............0')

        img_raw = bridge.imgmsg_to_cv2(img_msg,'passthrough')
        #img_rect = cv2.remap(img_raw,map1,map2,interpolation=cv2.INTER_LINEAR)
        img_rect = img_raw
        #Lidar  to Cam 3d.
        pointcloud = pc2.read_points(lidar_msg, skip_nans=True, field_names=("x", "y", "z"))
        pc_list = []
        for p in pointcloud:
            # print('p:{}'.format([p[0],p[1],p[2]]))
            if filter_xyz(p):
                pc_list.append( [p[0],p[1],p[2]] )
            # pc_list.append( [p[0],p[1],p[2]] )
        print('atfer filter points num:{}'.format(len(pc_list)))
        print('sync!...............1')

        pt_2ds = []
        img_draw = img_rect[:]
        #font = cv2.FONT_HERSHEY_COMPLEX
        font = cv2.FONT_HERSHEY_SCRIPT_COMPLEX
        font = cv2.FONT_HERSHEY_TRIPLEX
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img_draw,"Check the lidar points projected to image, especially points on edge.",(40, 40),font,0.7, (0, 0, 255), 2)
        cv2.putText(img_draw,"Points with similar color have similar depth.",(40,60),font,0.7,(0,0,255),2)
        cv2.putText(img_draw,"Notice that the camera principle point's 3d-line may not pass lidar center,",(40,80),font,0.7,(0,0,255),2)
        cv2.putText(img_draw,"  so that you may see some points behind object in image",(40,100),font,0.7,(0,0,255),2)
        cv2.putText(img_draw,"  which will not affect the precision of calibrated params.",(40, 120),font,0.7, (0, 0, 255), 2)
        cv2.putText(img_draw,"Press Q to quit.",(40, 140),font,0.7, (50, 50, 50), 2)

        total_pt2ds = 0
        # print(T_cam_lidar)
        T_final =  T_cam_lidar.dot(get_T_finetune())
        print('T_finetune={}'.format(T_finetune))
        print('T_final={}'.format(T_final))
        # T_final =  T_cam_lidar
        for point in pc_list:
            pt = np.array([0.0,0,0,1])
            pt[0] = point[0]
            pt[1] = point[1]
            pt[2] = point[2]
            #if(pt[1]<0):
            #    continue

            cam_coord = T_final.dot(pt.T).T   #lidar-->camera
            pt2 = np.array([0.0,0,0])
            pt2[0] = cam_coord[0]
            pt2[1] = cam_coord[1]
            pt2[2] = cam_coord[2]
            if(pt2[2]<0):
                continue
            pt_img = (camNewK.dot(pt2.T)).T  #camera->image
            pt_img_final = pt_img/pt_img[2]
            p2d = pt_img_final
            if(p2d[0]>=0 and p2d[0]<width and p2d[1]>=0 and p2d[1]<height):
                total_pt2ds+=1

            dist = math.sqrt(pt[0]*pt[0]+pt[1]*pt[1]+pt[2]*pt[2])
            cv2.circle(img_draw, (int(pt_img_final[0]),int(pt_img_final[1])), 1,(int(dist*240)%255, int(dist*30)%255, int(dist*4)%255 ))
            #投影到像平面。
            pt_2ds.append(pt_img_final)


        #for p2d in pt_2ds:
        #    #图像上绘制2d点 。
        print('total_p2ds in image:'+str(total_pt2ds))
        cv2.imshow('Reprojected image',img_draw)
        key_input = cv2.waitKey(1059)
        print('!!!!!!!!!!!!!!total_p2ds in image:'+str(total_pt2ds))
        # fine tune.手动调整,使得参数更精准
        global yaw,pitch,roll,x__,y__,z__
        if(key_input ==  ord('q')):
            yaw += 0.25
            refresh_T_finetune()
        if(key_input ==  ord('a')):
            yaw -= 0.25
            refresh_T_finetune()
        if(key_input ==  ord('w')):
            pitch+=0.5
            refresh_T_finetune()
        if(key_input ==  ord('s')):
            pitch-=0.5
            refresh_T_finetune()
        if(key_input ==  ord('e')):
            roll += 0.25
            refresh_T_finetune()
        if(key_input ==  ord('d')):
            roll -= 0.25
            refresh_T_finetune()

        if(key_input ==  ord('u')):
            x__+= 0.5  # 5mm
            refresh_T_finetune()
        if(key_input ==  ord('j')):
            x__-= 0.5  # 5mm
            refresh_T_finetune()
        if(key_input ==  ord('i')):
            y__+= 0.005  # 5mm
            refresh_T_finetune()
        if(key_input ==  ord('k')):
            y__-= 0.005
            refresh_T_finetune()
        if(key_input ==  ord('o')):
            z__+= 0.005
            refresh_T_finetune()
        if(key_input ==  ord('l')):
            z__ -= 0.005
            refresh_T_finetune()
        if(key_input == ord(' ')):
            echo_T_full_and_write()
    except ValueError:
        print ("Error occured in callback!")
    print("get out of callback")
#   except Exception ,e:
#     print ("Error occured in callback!")
#     print e
#     return

image_topic,lidar_topic=None,None

image_topic = '/image_raw'
lidar_topic = '/sensing/lidar/top/rectified/pointcloud'

# image_topic = '/image_raw_tmp'
# lidar_topic = '/pointcloud_tmp'


class visual_lidar_cam(Node):
    def __init__(self):
        super().__init__('visual_lidar_cam_node')
        
        self.publisher_lidar  = self.create_publisher(PointCloud2, '/pointcloud_tmp', 10)
        self.publisher_camera = self.create_publisher(Image, '/image_raw_tmp', 10)

        qos_profile = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        self.subscription_lidar = self.create_subscription(
            PointCloud2,
            '/sensing/lidar/top/rectified/pointcloud',
            self.lidar_callback,
            qos_profile)
        # self.subscription_lidar  # prevent unused variable warning

        self.subscription_camera = self.create_subscription(
            Image,
            '/image_raw',
            self.img_callback,
            qos_profile)
        # self.subscription_camera  # prevent unused variable warning


        img_sub = message_filters.Subscriber(self, Image,image_topic,qos_profile=qos_profile)
        lidar_sub = message_filters.Subscriber(self, PointCloud2,lidar_topic,qos_profile=qos_profile)
        synchronizer = message_filters.ApproximateTimeSynchronizer
        # ts = synchronizer([img_sub, lidar_sub], 20, 0.01, allow_headerless=True)
        
        ts = synchronizer([img_sub, lidar_sub], 20,1,allow_headerless=True)
        ts.registerCallback(self.lidar_cam_sync)

        lidar_sub.registerCallback(self.lidar_callback_t)
        img_sub.registerCallback(self.img_callback_t)
    
    def img_callback_t(self, msg):
        print(msg.header.stamp)
        print("img————————————————————————————————")

    def lidar_callback_t(self, msg):
        print(msg.header.stamp)
        print("lidar——————————————————————————————")

    def lidar_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_lidar.publish(msg)
        # print("lidar...")

    def img_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_camera.publish(msg)
        # print("img...")

    def lidar_cam_sync(self, Image, PointCloud2):
        # __callback_func__(img_msg,lidar_msg)
        __callback_func__(Image,PointCloud2)
        print(Image.header.stamp)
        print(PointCloud2.header.stamp)
        print("lidar and camera sync+++++++++++++++++++++++++++++++++++++++++++++++++++++==")    

def main(args=None):
    """python visualize_on_bag.py [0|1|2|3] """
    rclpy.init(args=args)

    visual_lidar_cam_node = visual_lidar_cam()
    
    global camid

    camid = int(sys.argv[1])
    # print(camid)
    set_in_ext_param(camid)
    print ('init node!')
   

    
    # img_sub = message_filters.Subscriber(visual_lidar_cam_node,image_topic, Image)
    # lidar_sub = message_filters.Subscriber(visual_lidar_cam_node,lidar_topic, PointCloud2)
    # print ('init subscribers!')
    # ts = message_filters.ApproximateTimeSynchronizer([img_sub, lidar_sub], 20, 0.01, allow_headerless=True)
    # print ('register callback!!')
    # ts.registerCallback(__callback_func__)
    # print ('spin!')
    rclpy.spin(visual_lidar_cam_node)
   


if __name__ == '__main__':
    main()


















    







