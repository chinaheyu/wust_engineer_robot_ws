#!/home/dji/miniforge3/bin/python3.8
import rospy
import rospkg
import os
os.chdir(rospkg.RosPack().get_path('wust_auto_align') + '/scripts')
import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import numpy as np
import pyrealsense2 as rs
import time
import json
import pathlib
from geometry_msgs.msg import Twist

rospy.init_node("wust_auto_align", anonymous=True)

vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10, tcp_nodelay=True)
vel_msg = Twist()

show_result = False

params = {'hue_low': 8, 'saturation_low': 110, 'value_low': 0,
          'hue_high': 30, 'saturation_high': 255, 'value_high': 255,
          'min_area_size': 5000, 'max_area_size': 75000, 'offset_x': 350,
          'offset_y': 0, 'kp': 0.008, 'kd': 0.003, 'max_speed': 0.8,
          'forward_velocity': 0.2}

params_file_path = pathlib.Path('params.json')

if params_file_path.exists():
    with params_file_path.open('r') as fp:
        params = json.load(fp)
else:
    print('Cannot find params.json, use default.')
    with params_file_path.open('w') as fp:
        json.dump(params, fp)

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.color, 640, 360, rs.format.bgr8, 60)

while True:
    try:
        rospy.loginfo('try to connect camera.')
        pipeline.start(config)
    except RuntimeError as e:
        rospy.loginfo(f'error: {e}')
    else:
        rospy.loginfo('connect camera successful.')
        break
    time.sleep(0.5)

thresh_low = (params['hue_low'], params['saturation_low'], params['value_low'])
thresh_high = (params['hue_high'], params['saturation_high'], params['value_high'])
min_area_size = params['min_area_size']
max_area_size = params['max_area_size']
offset_x = params['offset_x']
offset_y = params['offset_y']
kp = params['kp']
kd = params['kd']
max_speed = params['max_speed']
forward_velocity = params['forward_velocity']
last_error = 0

def limit_speed(speed):
    if speed > max_speed:
        return max_speed
    elif speed < -max_speed:
        return -max_speed
    return speed

def find_ore(color_image):
    cv2.GaussianBlur(color_image, (5, 5), 3, dst=color_image)
    hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
    ore_image = cv2.inRange(hsv_image, thresh_low, thresh_high)
    kernal = cv2.getStructuringElement(0, (3, 3))
    cv2.erode(ore_image, kernal, dst=ore_image)
    cv2.dilate(ore_image, kernal, dst=ore_image)

    contours, hierarchy = cv2.findContours(ore_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        area_size = list(map(cv2.contourArea, contours))
        max_size = max(area_size)
        max_area_index = area_size.index(max_size)
        if min_area_size < max_size < max_area_size:
            box = cv2.boundingRect(contours[max_area_index])
            return box

    return None

rospy.loginfo('auto align start.')
undetected_count = 0
st = time.time()
try:
    while not rospy.is_shutdown():
        # get and align frames
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # to numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # detect ore
        bbox = find_ore(color_image)
        if bbox is not None:
            undetected_count = 0
            x, y, w, h = bbox
            center_x = round(x + w / 2)
            center_y = round(y + h / 2)

            # print(center_x)

            cv2.rectangle(color_image, (x, y), (x + w, y + h), (73, 245, 189), thickness=2)

            cv2.circle(color_image, (center_x, center_y), 10, (73, 245, 189), cv2.FILLED)
            cv2.putText(color_image, "Target detected.", (0, 24), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 121, 242), 2)
            cv2.putText(color_image, f"x: {center_x}", (x, y - 48), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 121, 242), 2)
            cv2.putText(color_image, f"y: {center_y}", (x, y - 12), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 121, 242), 2)
            
            error = offset_x - center_x
            # publist msg
            vel_msg.linear.x = forward_velocity
            vel_msg.linear.y = limit_speed(kp * error + kd * (error - last_error))
            vel_msg.angular.z = 0
            vel_publisher.publish(vel_msg)
            last_error = error
        else:
            undetected_count = undetected_count + 1
            if undetected_count > 8:
                vel_msg.linear.x = forward_velocity
                vel_msg.linear.y = 0
                vel_msg.angular.z = 0
                vel_publisher.publish(vel_msg)
                last_error = 0


        # calc fps
        et = time.time()
        cv2.putText(color_image, f"FPS: {1 / (et - st):.2f}", (450, 24), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 121, 242), 2)
        st = et

        # show result
        if show_result:
            cv2.imshow('RealSense', color_image)
            if cv2.waitKey(1) == ord('q'):
                break


finally:
    pipeline.stop()


cv2.destroyAllWindows()

