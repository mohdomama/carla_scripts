""" Control ego-vehicles using keyboard.

    This script enables control of the ego-vehicles. It requires launch_ego_vehicle.py to be running simultaneously in
    a different shell. It also requires the ego-vehicles' IDs, obtained from launch_ego_vehicle.py after execution. 

    CONTROLS:
        W - increase linear velocity
        S - decrease linear velocity
        A, D - increase angular velocity left/right respectively
        B - break
        R - toggle reverse (used in conjuntion with W and S)
        L - print center ego_vehicle's location

    USAGE:
        python3 control.py 
"""

from teleop import Keyboard
import numpy as np
import carla
from launch_ego_vehicle import ego_transforms

def action_w(control):
    if control[0] < 1: control[0] += 0.05
    return control

def action_a(control):
    if control[1] > -1: control[1] -= 0.3
    return control

def action_s(control):
    if control[0] > 0: control[0] -= 0.05
    return control

def action_d(control):
    if control[1] < 1: control[1] += 0.3
    return control

def action_r(control):
    # press once to enable reverse driving, press again to disable
    if control[3] == 1.0:
        control[3] = 0.0
    else:
        control[3] = 1.0
    return control

def action_b(control):
    if control[1] < 1: control[2] += 0.3
    return control


ACTION_KEYS = {
    'w': action_w,
    'a': action_a,
    's': action_s,
    'd': action_d,
    'r': action_r,
    'b': action_b,
}


def main():
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    keyboard = Keyboard(0.05)
    control = np.array([0.0, 0.0, 0.0, 0.0])
    num_vehicles = len(ego_transforms)
    ego_ids = [int(input(f'Enter Ego {x} Vehicle ID: ')) for x in range(num_vehicles)]
    actor_list = world.get_actors()
    ego_vehicles = [actor_list.find(x) for x in ego_ids]

    while True:
        key = keyboard.read()
        if key in ACTION_KEYS:
            ACTION_KEYS[key](control)
        elif (key == 'h'):
            control = np.array([0.0, 0.0, 0.0, 0.0])
        elif (key == '\x03'):
            print('Control C')
            break
        elif (key=='l' or key=='L'):
            print(ego_vehicles[0].get_transform())
        else:
            # Steer and break reset
            control[1] = 0
            control[2] = 0

        print(control)
        for i in range(num_vehicles):
            ego_vehicles[i].apply_control(carla.VehicleControl(throttle=control[0], steer=control[1], brake=control[2], reverse=bool(control[3])))

if __name__=='__main__':
    main()