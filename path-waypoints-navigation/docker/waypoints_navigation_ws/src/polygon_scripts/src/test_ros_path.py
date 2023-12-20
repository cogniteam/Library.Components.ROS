#!/usr/bin/env python3
import sys
import time
import numpy as np
import math
from enum import Enum
from struct import pack, unpack
import operator
import rospy
from polygon_msgs.msg import *
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply
import ast


from nav_msgs.msg import Odometry, Path

class Directions(Enum):
    Y_minus_direction = -180
    Y_Direction = 0
    X_minus_direction = -90
    X_direction = 90

class MotionState(Enum):
    Setup = 1
    Homing = 2
    IDLE = 3
    DRIVING = 4
    DISCRETE = 5
    Decalaration = 6
    JOYSTICK = 7
    Recover = 8
    Error = 9

def get_angle_from_two_points(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    dy = y2 - y1
    dx = x2 - x1

    if dx == 0:  # on y axis
        if y2 > y1:  # y direction
            return 90
        else:  # -y direction
            return 270
    elif dy == 0:  # on x axis
        if x2 > x1:  # x direction
            return 0
        else:
            return 180
    else:
        angle = np.rad2deg(np.arctan(dy/dx))
        if dx > 0:
            if dy > 0:
                return angle
            else:
                return 360 + angle
        else:  # dx < 0
            if dy > 0:
                return 180 + angle
            else:
                return 180 + angle

def from_y_angle_to_x_angle(angle):
    '''This method rotates angle from angle scheme with y is 0, to x is 0.
    Parameters:
    ----------
    angle: integer
        Angle in new schem, in degrees ((-180) - 180) 0 is along y axis.
    Returns:
    ---------
    integer
        angle in degrees (0-359), 0 is along x axis.
    '''
    if angle > 90 and angle <= 180:
        return 360 - angle + 90
    else:
        return 90 - angle

def from_x_angle_to_y_angle(angle):
    '''This method rotates angle from angle scheme with x is 0, to y is 0.
    Parameters:
    ----------
    angle: integer
        angle in degrees (0-360), 0 is along x axis.
    Returns:
    ---------
    integer
        Angle in new schem, in degrees ((-180) - 180) 0 is along y axis.
    '''
    if angle > 270 and angle < 360:
        return 360+(90-angle)
    return 90 - angle

def get_direction(p1, p2):
    ''' This function returns the direction of SDV movment
    Parameters
    ----------
    p1:
        represented by [x, y]
    p2:
        represented by [x, y]
    Returns
    -------
    Direction:
        movement direction
    '''
    if p1[1] == p2[1]:
        if p1[0] > p2[0]:
            return Directions.X_minus_direction.value
        else:
            return Directions.X_direction.value
    elif p1[0] == p2[0]:
        if p1[1] > p2[1]:
            return Directions.Y_minus_direction.value
        else:
            return Directions.Y_Direction.value
    else:
        print('Diagonal path points. p1: ' + str(p1) + '. p2: ' + str(p2))
        
        return from_x_angle_to_y_angle(get_angle_from_two_points((p1[0], p1[1]), (p2[0], p2[1])))



# path =  [(7.470, 2.200, -10, 0, 0),
#             (10.570, 2.200, 1.000, 0, 0),
#             (10.570, 3.700, -10, 0, 0),
#             (11.430, 3.750, -10, 0, 0),
#             (11.430, 3.760, -10, 0, 1)]

# path = [(4.010, 3.560, -10, 0, 0),
#             (11.010, 3.560, 1.000, 0, 0),
#             (11.010, 1.560, 1.000, 0, 0),
#             (10.010, 1.560, -10, 0, 0),
#             (9.610, 1.460, -10, 0, 0),
#             (4.710, 1.250, -10, 0, 0)]
# path = [(7.070, 5.520, -10, 0, 0),
#         (7.070, 3.420, 2.000, 0, 0),
#         (9.070, 3.420, -10, 0, 0),
#         (11.070, 2.920, -10, 0, 0),
#         (13.370, 2.920, -10, 0, 0),
#         (13.470, 3.320, -10, 0, 0),
#         (13.470, 3.910, -10, 0, 0),
#         (13.470, 3.920, -10, 0, 1)]
# path = [(7.070, 5.520, 0),
#         (7.070, 3.420, 2.0),
#         (9.070, 3.420, 0),
#         (11.070, 2.920, 0),
#         (13.370, 2.920, 0),
#         (13.470, 3.320, 0),
#         (13.470, 3.910, 0),
#         (13.470, 3.920, 0)]
# path = [(6.950, 5.590, -10, 0, 0),
#         (6.950, 5.290, -10, 0, 0),
#         (7.450, 3.290, -10, 0, 0),
#         (9.050, 3.190, -10, 0, 0),
#         (1.0250, 2.890, -10, 0, 0),
#         (1.3350, 2.890, -10, 0, 0),
#         (1.3450, 3.290, -10, 0, 0),
#         (1.3470, 3.910, -10, 0, 0),
#         (1.3470, 3.920, -10, 0, 1)]



# task_path = [(4820, 1930), (6620, 1930, -10, 0, 0), (6620, 3030, 1000, 0, 0), (10220, 3030, -10, 0, 0), (12430, 2250, -10, 0, 0)]

# task_path = [(4720, 1480), (6000, 1480, -10, 0, 0), (6000, 3000, -10, 0, 0), (8000, 3000, -10, 0, 0)]

# task_path = [(5190, 1300), (7100, 1300, 1000, 0, 0), (7100, 2800, -10, 0, 0), (8200, 2800, -10, 0, 0)]

# task_path = [(5000, 1110), (7500, 1110, 1100, 0, 0), (7500, 3700, 1100, 0, 0), (11000, 3700, -10, 0, 0)]

# task_path = [(3.320, 2.620, -10, 0, 0), (3.320, 3.120, -10, 0, 0),
#             (6.520, 3.120, -10, 0, 0),
#             (11.400, 2.760, -10, 0, 0),
#             (11.400, 2.750, -10, 0, 1)]


# path = [(4.010, 3.560, -10, 0, 0), \
#         (11.010, 3.560, 1.000, 0, 0), \
#         (11.010, 1.560, 1.000, 0, 0), \
#         (10.010, 1.560, -10, 0, 0), \
#         (9.610, 1.460, -10, 0, 0), \
#         (4.710, 1.250, -10, 0, 0)]




def get_s(path, final_direction=None):
    
    keys = ['x', 'y', 'distanceRadius', 'direction', 'turnPoint', 'stopTurnPoint', 'originCenter','radiusDir']
    segment_length=0
    segments = []
    # idx = 1
    prev_radius = 0
    num_of_points = len(path)
    new_path = []
    wp = []
    for i in range(1, num_of_points):
        p1 = path[i - 1]
        p2 = path[i]
        direction = get_direction(p1, p2)
        x_i=p1[0]
        y_i=p1[1]
        prev_direction = np.deg2rad(from_y_angle_to_x_angle(direction))
        wp.append([p1[0], p1[1], prev_direction])
        radius = path[i][2]
        
        contour_turn = None
        turn_x = -1
        turn_y = -1
        (stop_turn_x, stop_turn_y) = (-1,-1)
        if radius > 0:
            turn_x = p2[0] - radius*np.cos(prev_direction)
            turn_y = p2[1] - radius*np.sin(prev_direction)
            
        center_of_origin = None
        radius_dir = None
        if prev_radius > 0:
            stop_turn_y = p1[1] + prev_radius*np.sin(prev_direction)
            stop_turn_x = p1[0] + prev_radius*np.cos(prev_direction)
            x_i = stop_turn_x
            y_i = stop_turn_y
            first_turn_x = segments[-1]['turnPoint'][0]
            first_turn_y = segments[-1]['turnPoint'][1]
            print(first_turn_x)
            print(first_turn_y)
            print(stop_turn_x)
            print(stop_turn_y)
            new_path[i - 2][1][0] = first_turn_x
            new_path[i - 2][1][1] = first_turn_y
            x1 = np.sign(stop_turn_x-first_turn_x) * prev_radius + first_turn_x
            y1 = np.sign(stop_turn_y-first_turn_y) * prev_radius + first_turn_y
            if abs(p2[0] - p1[0]) == 0:
                center_of_origin = (first_turn_x, y1)

                if first_turn_x > stop_turn_x:
                    if first_turn_y > stop_turn_y:
                        radius_dir =  (-math.pi/2, -math.pi)
                    elif first_turn_y < stop_turn_y:
                        radius_dir =  (math.pi/2, math.pi)
                elif first_turn_x < stop_turn_x:
                    if first_turn_y > stop_turn_y:
                        radius_dir =  (math.pi/2, 0)
                    elif first_turn_y < stop_turn_y:
                        radius_dir =  (-math.pi/2, 0)

            elif abs(p2[1] - p1[1]) == 0:
                center_of_origin = (x1, first_turn_y)

                if first_turn_x > stop_turn_x:
                    if first_turn_y > stop_turn_y:
                        radius_dir =  (0, -math.pi/2)
                    elif first_turn_y < stop_turn_y:
                        radius_dir =  (0, math.pi/2)

                elif first_turn_x < stop_turn_x:
                    if first_turn_y > stop_turn_y:
                        radius_dir =  (-math.pi, -math.pi/2)
                    elif first_turn_y < stop_turn_y:
                        radius_dir =  (math.pi, math.pi/2)

        
        prev_radius = radius
        new_path.append([[x_i, y_i],[p2[0], p2[1]], prev_direction])
        segment = [p2[0], p2[1], radius, prev_direction, (turn_x, turn_y),(stop_turn_x, stop_turn_y) ,center_of_origin, radius_dir]
        
        segments.append((dict(map(list, list(zip(keys, segment))))))
    
    wp.append([p2[0], p2[1], prev_direction])
    for i in range(len(segments)):
        if segments[i]['originCenter'] is not None:
            if new_path[i-1][0] != 'arc':
                new_path.insert(i,['arc',segments[i]['originCenter'],segments[i-1]['distanceRadius'],segments[i]['radiusDir']])
            else:
                new_path.insert(i+1,['arc',segments[i]['originCenter'],segments[i-1]['distanceRadius'],segments[i]['radiusDir']])
    # print(wp)
    return new_path, wp



def get_points(p):
    points=[]
    for item in p:
        # print(item)
        if isinstance(item[0], list):
            # print('&&&&&&&&&&&&&&&&&&&&&&&&&&&&')
            x_i = item[0][0]
            y_i = item[0][1]
            x_f = item[1][0]
            y_f = item[1][1]
            sample_rate = int(math.hypot(x_i-x_f,y_i-y_f) / line_rate)
            if sample_rate < 1:
                sample_rate = 2
            c = list(map(list, np.linspace((x_i, y_i), (x_f, y_f),sample_rate)))
            for i in c:
                i.append(item[2])
                # print(i[0],i[1],i[2])
            if c:
                points.extend(c)
                
        
        elif isinstance(item[0], str):
            # print('@@@@@@@@@@@@@@@@@')
            # print(item)
            a = item[1][0]
            b = item[1][1]
            r = item[2]

            init_angle = item[3][0]
            # print(init_angle)
            final_angle = item[3][1]
            # print(final_angle)

            if operator.lt(init_angle, final_angle):
                comp_func = operator.lt
                oper_func = operator.iadd
            else:
                comp_func = operator.gt
                oper_func = operator.isub
            # print(comp_func(init_angle, final_angle))
            # print(init_angle, final_angle)
            #The lower this value the higher quality the circle is with more points generated
            stepSize = arc_rate / 1.185

            # if sample_rate < 1:
            #     sample_rate = 2
            # stepSize = 0.1

            #Generated vertices
            positions = []
            # print(points[-1])
            last_point_angle = points[-1][-1]
            t = init_angle
            
            while comp_func(t, final_angle):
                last_point_angle = oper_func(last_point_angle,stepSize)
                positions.append([r * math.cos(t) + a, r * math.sin(t) + b , last_point_angle])
                
                # print(r * math.cos(t) + a, r * math.sin(t) + b , last_point_angle)
                t = oper_func(t,stepSize)
            points.extend(positions)
    return points





def get_path(points,wp):
    

    
    intersect = Path()
    intersect.header.frame_id = "map"
    intersect.header.stamp = rospy.Time(0)
    for point in points:
        pose = PoseStamped()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.z = 0
        quaternion = tf.transformations.quaternion_from_euler(
            0, 0, point[2])
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        intersect.poses.append(pose)
    with open(str(file_path) + 'path.txt','ab') as f:
        f.truncate(0)
        path = str.encode(str(intersect))
        # f.write(intersect)
        f.write(path)
        f.close()

    route = Path()
    route.header.frame_id = "map"
    route.header.stamp = rospy.Time(0)
    for w in wp:
        pose = PoseStamped()
        pose.pose.position.x = w[0]
        pose.pose.position.y = w[1]
        pose.pose.position.z = 0
        quaternion = tf.transformations.quaternion_from_euler(
            0, 0, w[2])
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        route.poses.append(pose)
    
    while True:
        pub_intersect.publish(intersect)
        pub_route.publish(route)
        time.sleep(1)
    # print(msg)
# p=get_s(path)
# po=get_points(p)
# get_path(po)
# print(po)

if __name__ == "__main__":

       
    try:
        # if len(sys.argv) < 3:
        #     print("usage: rosrun polygon_scripts test_ros_path.py line_rate arc_rate")
        #     print("Example: rosrun polygon_scripts test_ros_path.py 0.15 0.1")
        #     exit()
        # else:
        #     line_rate = float(sys.argv[1])
        #     arc_rate = float(sys.argv[2]) 

        rospy.init_node("path_creator")
        maestro_path = rospy.get_param('~maestro_path')['maestro_path']
        line_rate = rospy.get_param('~line_rate')
        arc_rate = rospy.get_param('~arc_rate')
        file_path = rospy.get_param('~file_path')
        # f = open('/home/musashi/rosmusashi/musashi_ws/src/polygon_scripts/src/maestro_path.txt','r+')
        # path = f.read()
        path = ast.literal_eval(maestro_path)
        print(path)
        
        (p, w)=get_s(path)
        po=get_points(p)

        pub_intersect = rospy.Publisher("/waypoints_route", Path, queue_size=1)
        pub_route = rospy.Publisher("/wp_route", Path, queue_size=1)
        # rospy.Timer(rospy.Duration(1), get_path)
        
        get_path(po, w)
        

        # rospy.spin()


    # rospy.ROSInterruptException
    except Exception as e:
        print("except-------------------------------------------------------------" + str(e))

    finally:
        exit()