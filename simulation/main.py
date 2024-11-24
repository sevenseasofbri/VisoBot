import time
import numpy as np
import pickle
import sys
import os
import pybullet as p
from stretch import *
from utils.tools import *
from grid import StaticGrid
from global_planner import *
from move import follow_waypoints
from trajectory import smooth_trajectory_cubic

p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
p.setGravity(0, 0, -9.81)

mobot = init_scene(p, mug_random=False)
object_ids = get_object_ids()

# Init static grid
grid_size = (51, 51)
cell_size = 0.2 # 20 cm
grid = StaticGrid(grid_size=grid_size, cell_size=cell_size)

grid.update_grid_with_objects(object_ids=object_ids)

# print("Grid initialized!")
# grid.print_grid()

# start_coordinate = (-0.8, 0)
# end_coordinate = (1.45, -1.68)

start_idx = grid.world_to_grid(-0.8, 0)
end_idx = grid.world_to_grid(-1.70, -3.70)
print(start_idx)
print(end_idx)

grid.print_grid()

path = astar(grid=grid, start=start_idx, end=end_idx)
print(path)

# Note: Always mark custom cells AFTER running astar...
grid.mark_custom_cell(start_idx, 3)
grid.mark_custom_cell(end_idx, 3)
grid.mark_custom_cells(path, 2) 

grid.mark_custom_cell(start_idx, 3)
grid.mark_custom_cell(end_idx, 3)

grid.print_grid()

# get path in real-world coordinates
waypoints = [grid.grid_to_world(cell[0], cell[1]) for cell in path]

print(waypoints)

smoothed_waypoints = smooth_trajectory_cubic(waypoints, num_points=200)

# Todo trajectory smoothing
    
forward=0
turn=0
speed=10
up=0
stretch=0
gripper_open=0
roll=0
yaw=0

mobot.get_observation()

total_driving_distance = 0
previous_position, _, _ = get_robot_base_pose(p, mobot.robotId)
current_position = previous_position

constraint = None

navi_flag = False
grasp_flag = False

while (1):
    time.sleep(1./240.)
    # keys = p.getKeyboardEvents()

    # for k,v in keys.items():
    #     # moving
    #     if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
    #         turn = -1
    #     if (k == p.B3G_RIGHT_ARROW and (v&p.KEY_WAS_RELEASED)):
    #         turn = 0
    #     if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_TRIGGERED)):
    #         turn = 1
    #     if (k == p.B3G_LEFT_ARROW and (v&p.KEY_WAS_RELEASED)):
    #         turn = 0
    #     if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_TRIGGERED)):
    #         forward=1
    #     if (k == p.B3G_UP_ARROW and (v&p.KEY_WAS_RELEASED)):
    #         forward=0
    #     if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_TRIGGERED)):
    #         forward=-1
    #     if (k == p.B3G_DOWN_ARROW and (v&p.KEY_WAS_RELEASED)):
    #         forward=0

    #     # lifting
    #     if (k == ord('z') and (v & p.KEY_WAS_TRIGGERED)):
    #         up = 1
    #     if (k == ord('z') and (v & p.KEY_WAS_RELEASED)):
    #         up = 0
    #     if (k == ord('x') and (v & p.KEY_WAS_TRIGGERED)):
    #         up = -1
    #     if (k == ord('x') and (v & p.KEY_WAS_RELEASED)):
    #         up = 0

    #     # stretching
    #     if (k == ord('a') and (v & p.KEY_WAS_TRIGGERED)):
    #         stretch = -1
    #     if (k == ord('a') and (v & p.KEY_WAS_RELEASED)):
    #         stretch = 0
    #     if (k == ord('d') and (v & p.KEY_WAS_TRIGGERED)):
    #         stretch = 1
    #     if (k == ord('d') and (v & p.KEY_WAS_RELEASED)):
    #         stretch = 0

    #     # roll
    #     if (k == ord('r') and (v & p.KEY_WAS_TRIGGERED)):
    #         roll = 1
    #     if (k == ord('r') and (v & p.KEY_WAS_RELEASED)):
    #         roll = 0
    #     if (k == ord('f') and (v & p.KEY_WAS_TRIGGERED)):
    #         roll = -1
    #     if (k == ord('f') and (v & p.KEY_WAS_RELEASED)):
    #         roll = 0

    #     # yaw
    #     if (k == ord('y') and (v & p.KEY_WAS_TRIGGERED)):
    #         yaw = 1
    #     if (k == ord('y') and (v & p.KEY_WAS_RELEASED)):
    #         yaw = 0
    #     if (k == ord('h') and (v & p.KEY_WAS_TRIGGERED)):
    #         yaw = -1
    #     if (k == ord('h') and (v & p.KEY_WAS_RELEASED)):
    #         yaw = 0


    #     # gripper
    #     if (k == ord('q') and (v & p.KEY_WAS_TRIGGERED)):
    #         gripper_open = -1
    #     if (k == ord('q') and (v & p.KEY_WAS_RELEASED)):
    #         gripper_open = 0
    #     if (k == ord('e') and (v & p.KEY_WAS_TRIGGERED)):
    #         gripper_open = 1
    #     if (k == ord('e') and (v & p.KEY_WAS_RELEASED)):
    #         gripper_open = 0

    # base_control(mobot, p, forward, turn)
    # arm_control(mobot, p, up, stretch, roll, yaw)

    # if gripper_open == 1:
    #     constraint = attach(21, mobot.robotId, 18)
    # elif gripper_open == -1:
    #     detach(constraint)
    #     constraint = None
    
    mobot.get_observation()

    follow_waypoints(p, mobot, smoothed_waypoints)
    
    current_position, _, _ = get_robot_base_pose(p, mobot.robotId)
    total_driving_distance += np.linalg.norm(np.array(current_position) - np.array(previous_position))
    previous_position = current_position

    if navi_flag == False:
        if current_position[0] > 1.6 and current_position[1] > -0.35:
            print("Reached the goal region! Total driving distance: ", total_driving_distance)
            break
            #navi_flag = True
        else:
            pass
            # print("Total driving distance: ", total_driving_distance)
            # print("Current position: ", current_position)
    else:
        pass
        #print("Reached the goal region! Total driving distance: ", total_driving_distance)
    
    
    if grasp_flag == False:
        mug_position = get_mug_pose(p)
        #print("Mug position: ", mug_position)

        if mug_position[0] > 3.3 and mug_position[0] < 3.5 \
            and mug_position[1] > -0.17 and mug_position[1] < 0.25 \
            and mug_position[2] > 0.71 and mug_position[2] < 0.75:
            print("Mug is in the drawer!")
            grasp_flag = True
    else:
        print("Mug is in the drawer!")

    ee_position, _, _ = get_robot_ee_pose(p, mobot.robotId)
    #print("End-effector position: ", ee_position)
