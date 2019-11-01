#!/usr/bin/env
import heapq
import time
import rospy
from geometry_msgs.msg import Twist
import math
import matplotlib.pyplot as plt
import numpy as np

# time stamp
start_time = time.time()

# resolution
res = 1

def ros_node(ipvar):
    rospy.init_node('turtlebot_project', anonymous=True)
    velo_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    turtlebot_velo = Twist()
    turtlebot_velo.linear.x = 0.0
    turtlebot_velo.angular.z = 0.0
    rate = rospy.Rate(10)    # Sampling rate
    x = 0
    n = 0
    while not rospy.is_shutdown():
        if (x <= 10):
            if (n == len(ipvar)):
                turtlebot_velo.linear.x = 0.0
                turtlebot_velo.angular.z = 0.0
                velo_pub.publish(turtlebot_velo)
                break

            x = float("{0:4f}".format(ipvar[n][2]/300))
            w = float("{0:4f}".format(ipvar[n][1]))
            turtlebot_velo.linear.x = x
            print("x vel: ", x)
            turtlebot_velo.angular.z = w
            print("w vel: ", w)
            velo_pub.publish(turtlebot_velo)
            if(x == 10):
                #print(n, turtlebot_velo.linear.x, turtlebot_velo.angular.z)
                n = n + 1
                x = 0
        x += 1
        rate.sleep()
        
# minkowski
extra = 30

# asking input for wheel velocities
#left_wheel_velo = int(input("enter left wheel RPM: "))
left_wheel_velo = 5
#right_wheel_velo = int(input("enter right wheel RPM: "))
right_wheel_velo = 10

# asking input for star coordinates
startx = int(input("please enter start point x coordinate: "))
starty = int(input("please enter start point y coordinate: "))

# asking input for goal coordinates
goalx = int(input("please enter goal point x coordinate: "))
goaly = int(input("please enter goal point y coordinate: "))

# start node
start = (int(startx / res), int(starty / res))
start_node = (startx, starty)
plt.plot(start[0], start[1], "Dr")

# goal node
goal = (int(goalx / res), int(goaly / res))
goal_node = (goalx, goaly)
plt.plot(goal[0], goal[1], "Dr")

obstacle = np.zeros(shape=(int(1111 / res), int(1011 / res)))

obstacle_space = []


def get_motion_model(RPM1, RPM2, dt, theta):
    action_space = []

    L = 35.4

    steps = [[0, RPM1],
             [RPM1, 0],
             [RPM1, RPM1],
             [0, RPM2],
             [RPM2, 0],
             [RPM2, RPM2],
             [RPM1, RPM2],
             [RPM2, RPM1]]
    r = 3.8


    for motion in steps:
        ul = motion[1]
        ur = motion[0]

        thetadot = (r / L) * (ur - ul)
        dtheta = thetadot * dt

        change_theta = theta + dtheta
        ydot = (r / 2) * (ul + ur) * math.sin(change_theta)
        xdot = (r / 2) * (ul + ur) * math.cos(change_theta)

        dy = round(ydot * dt)
        dx = round(xdot * dt)

        vel_mag = math.sqrt((ydot) ** 2 + (xdot) ** 2)

        cost = float(math.sqrt(dy ** 2 + dx ** 2))

        action_space.append((dx, dy, cost, change_theta, dtheta, vel_mag))

    return action_space


for x in range(int(1110 / res)):
    for y in range(int(1010 / res)):

        C1 = (x - round(390/res))**2 + (y - round(960/res))**2 - ((40.5/res) + extra)**2
        if C1 <= 0:
            obstacle_space.append((x, y))

        C2 = (x - round(438 / res)) ** 2 + (y - round(736 / res)) ** 2 - ((40.5 / res) + extra) ** 2
        if C2 <= 0:
            obstacle_space.append((x, y))

        C3 = (x - round(390 / res)) ** 2 + (y - round(45 / res)) ** 2 - ((40.5 / res) + extra) ** 2
        if C3 <= 0:
            obstacle_space.append((x, y))

        C4 = (x - round(438 / res)) ** 2 + (y - round(274 / res)) ** 2 - ((40.5 / res) + extra) ** 2
        if C4 <= 0:
            obstacle_space.append((x, y))

# ########### RECTANGLE 1 ##############
        f1 = -y + (0/res) - extra
        f2 = y - (35/res) - extra
        f3 = -x + (685/res) - extra
        f4 = x - (1110/res) - extra

        if f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0:
            obstacle_space.append((x, y))

# ########### RECTANGLE 2 ##############

        f1 = -y + (35 / res) - extra
        f2 = y - (111 / res) - extra
        f3 = -x + (927 / res) - extra
        f4 = x - (1110 / res) - extra

        if f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0:
            obstacle_space.append((x, y))

# ########### RECTANGLE 3 ##############

        f1 = -y + (35 / res) - extra
        f2 = y - (93 / res) - extra
        f3 = -x + (779 / res) - extra
        f4 = x - (896 / res) - extra

        if f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0:
            obstacle_space.append((x, y))

# ########### RECTANGLE 4 ##############

        f1 = -y + (35 / res) - extra
        f2 = y - (187 / res) - extra
        f3 = -x + (474 / res) - extra
        f4 = x - (748 / res) - extra

        if f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0:
            obstacle_space.append((x, y))

# # ########### RECTANGLE 5 ##############
#         f1 = -y + (295.25/res) - extra
#         f2 = y - (412.25/res) - extra
#         f3 = -x + (1052/res) - extra
#         f4 = x - (1110/res) - extra
#
#         if f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0:
        #             obstacle_space.append((x, y))

# ########### RECTANGLE 6 ##############
        f1 = -y + (919/res) - extra
        f2 = y - (1010/res) - extra
        f3 = -x + (983/res) - extra
        f4 = x - (1026/res) - extra

        if f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0:
            obstacle_space.append((x, y))

# ########### RECTANGLE 7 ##############
        f1 = -y + (827/res) - extra
        f2 = y - (1010/res) - extra
        f3 = -x + (832/res) - extra
        f4 = x - (918/res) - extra

        if f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0:
            obstacle_space.append((x, y))

# ########### RECTANGLE 8 ##############
        f1 = -y + (621/res) - extra
        f2 = y - (697/res) - extra
        f3 = -x + (744/res) - extra
        f4 = x - (1110/res) - extra

        if f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0:
            obstacle_space.append((x, y))

# ########### RECTANGLE 9 ##############
        f1 = -y + (449/res) - extra
        f2 = y - (566/res) - extra
        f3 = -x + (1052/res) - extra
        f4 = x - (1110/res) - extra

        if f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0:
            obstacle_space.append((x, y))

# ########### RECTANGLE 10 ##############
        f1 = -y + (363/res) - extra
        f2 = y - (449/res) - extra
        f3 = -x + (1019/res) - extra
        f4 = x - (1110/res) - extra

        if f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0:
            obstacle_space.append((x, y))

# ########### RECTANGLE 11 ##############
        f1 = -y + (178.75/res) - extra
        f2 = y - (295.75/res) - extra
        f3 = -x + (1052/res) - extra
        f4 = x - (1110/res) - extra

        if f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0:
            obstacle_space.append((x, y))

# ########### RECTANGLE 12 ##############
        f1 = -y + (315/res) - extra
        f2 = y - (498/res) - extra
        f3 = -x + (438/res) - extra
        f4 = x - (529/res) - extra

        if f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0:
            obstacle_space.append((x, y))

# ########### RECTANGLE 13 ##############
        f1 = -y + (265/res) - extra
        f2 = y - (341/res) - extra
        f3 = -x + (529/res) - extra
        f4 = x - (712/res) - extra

        if f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0:
            obstacle_space.append((x, y))

# ########### RECTANGLE 14 ##############
        f1 = -y + (267/res) - extra
        f2 = y - (384/res) - extra
        f3 = -x + (784.5/res) - extra
        f4 = x - (936.5/res) - extra

        if f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0:
            obstacle_space.append((x, y))

# ########### TABLE SQUARE ##############
        f1 = -y + (751/res) - extra
        f2 = y - (910/res) - extra
        f3 = -x + (149.5/res) - extra
        f4 = x - (318.885/res) - extra

        if f1 <= 0 and f2 <= 0 and f3 <= 0 and f4 <= 0:
            obstacle_space.append((x, y))

# ########### TABLE CIRCLE LEFT ##############
        TCL = (x - round(149.5/res))**2 + (y - round(830.5/res))**2 - ((79.5/res) + extra)**2
        if TCL <= 0:
            obstacle_space.append((x, y))

# ########### TABLE CIRCLE RIGHT ##############
        TCR = (x - round(318.885 / res)) ** 2 + (y - round(830.5 / res)) ** 2 - ((79.5 / res) + extra) ** 2
        if TCR <= 0:
            obstacle_space.append((x, y))

        b1 = y - extra
        b2 = x- extra
        b3 = y - (1010 - extra)
        b4 = x - (1110 - extra)

        if (b1<= 0 or b2<= 0 or b3>= 0 or b4 >= 0):
            obstacle_space.append((x, y))

# ########### WALLS ##############
for i in range(int(1110 / res)):
    obstacle_space.append((i, 0))
    obstacle_space.append((i, round(1010 / res)))
for j in range(int(1010 / res)):
    obstacle_space.append((0, j))
    obstacle_space.append((round(1110 / res), j))

# extracting obstacle points
y_points = [y[1] for y in obstacle_space]
x_points = [x[0] for x in obstacle_space]

oy = []
ox = []

plt.scatter(x_points, y_points, color='r')
plt.xlim(0, 1110 / res)
plt.ylim(0, 1010 / res)

for i, j in obstacle_space:
    obstacle[i][j] = 1
    ox.append(i)
    oy.append(j)

start = time.time()

visit_x, visit_y = [], []
retrace = []

def Holonomic_astar(sx, sy, gx, gy):

    closed_list = []
    open_list = []
    goal_node = (0, (gx, gy), None, 0, 0, 0, 0)
    start_node = (0, (sx, sy), None, 0, 0, 0, 0)

    heapq.heappush(open_list, start_node)

    obstacle[start_node[1][0]][start_node[1][1]] = 1

    while len(open_list) > 0:
        current_node = heapq.heappop(open_list)
        heapq.heappush(closed_list, current_node)
        motion = get_motion_model(left_wheel_velo, right_wheel_velo, 0.5, current_node[4])

        visit_y.append(current_node[1][1])
        visit_x.append(current_node[1][0])

        curr_vel = current_node[5]
        curr_change_angle = current_node[4]
        retrace.append((current_node[1], current_node[2], curr_change_angle, curr_vel))

        if len(visit_x) % 2000 == 0 or len(visit_x) == 1:
            plt.plot(visit_x, visit_y, ".y")

        threshold = round(np.sqrt((current_node[1][0] - goal_node[1][0]) ** 2 + (current_node[1][1] - goal_node[1][1]) ** 2))

        if threshold <= 15:
            path = []
            l = len(retrace)
            current_change_angle = retrace[l - 1][3]
            current_pos = retrace[l - 1][0]
            current_vel = retrace[l - 1][2]

            path.append((current_pos, current_vel, current_change_angle))
            parent = retrace[l - 1][1]
            print("Goal found")

            while parent is not None:
                for i in range(l):
                    X = retrace[i]
                    if X[0] == parent:
                        current_pos = X[0]
                        parent = X[1]
                        path.append((current_pos, X[2], X[3]))
            return path[::-1]

        sub = []
        for new_position in motion:

            node_position = (current_node[1][0] + new_position[0], current_node[1][1] + new_position[1])
            node_position_cost = current_node[3] + new_position[2]
            heuristic_cost = (math.sqrt((goal_node[1][0] - node_position[0]) ** 2 + (goal_node[1][1] - node_position[1]) ** 2))

            final_cost = node_position_cost + heuristic_cost

            node_dtheta = new_position[4]
            node_change_angle = new_position[3]
            node_parent = current_node[1]
            node_vel_pub = new_position[5]

            maxy = (len(obstacle) - 1)
            maxx = (len(obstacle[0]) - 1)
            minx = 0
            miny = 0

            if node_position[0] > maxy:
                continue

            if node_position[0] < miny:
                continue

            if node_position[1] > maxx:
                continue

            if node_position[1] < minx:
                continue

            if obstacle[node_position[0]][node_position[1]] != 0:
                continue

            obstacle[node_position[0]][node_position[1]] = 1

            new_node = (final_cost, node_position, node_parent, node_position_cost, node_change_angle, node_vel_pub, node_dtheta)
            sub.append(new_node)
            heapq.heappush(open_list, new_node)


if goal_node in zip(ox, oy) or start_node in zip(ox, oy):
    if goal_node in zip(ox, oy):
        print("\nGoal node in obstacle space")
    if start_node in zip(ox, oy):
        print("\nStart node in obstacle space")
else:
    path = Holonomic_astar(startx, starty, goalx, goaly)
    if len(path) == 0:
        print(
            "\n path not found.")
    else:

        path_x, path_y = [], []

        for i in range(len(path)):
            path_x.append(path[i][0][0])
            path_y.append(path[i][0][1])
        print('path', path)
        plt.plot(path_x, path_y, "-k", linewidth=2)
        plt.plot(visit_x, visit_y, ".y")
        elapsed_time = time.time() - start_time
        print("\ntime elapsed: ", elapsed_time)
        plt.show()

ros_node(ros_input)
