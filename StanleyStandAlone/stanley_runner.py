import numpy as np
import carla
import time
from scene import setup_carla
from transforms import se3_to_components
import stanley
from cubic_spline_planner import calc_spline_course
from matplotlib import pyplot as plt


def get_current_speed(EGO):
    vel_ego = EGO.get_velocity()
    speed = np.linalg.norm(np.array([vel_ego.x, vel_ego.y]))
    return speed


def get_state(ego, tf_matrix):
    ego_tf = ego.get_transform().get_matrix()
    ego_tf_odom = np.linalg.pinv(tf_matrix) @ ego_tf
    x, y, z, roll, pitch, yaw = se3_to_components(ego_tf_odom)
    speed = get_current_speed(ego)
    return speed, x, -y, -yaw  # In RHCS


def main():
    FPS = 20
    ego, world, spectator, dummy = setup_carla(FPS)

    odom_tf = np.array(ego.get_transform().get_matrix())

    # Trajectories are in RHCS
    # Trajectory 1  
    path_x = np.array([0, 20, 22, 25, 30])
    path_y = np.array([0, 6, 6, 6, 6])

    # # Trajectory 2
    # path_x = np.array([0, 20, 22, 25, 30])
    # path_y = np.array([0, -6, -6, -6, -6])

    sp_x, sp_y, sp_yaw, _, _ = calc_spline_course(path_x, path_y, 0.2)

    desired_speed = 4
    # Getting the max steering angle
    # For each Wheel Physics Control, print maximum steer angle
    physics_control = ego.get_physics_control()
    max_steer_angle_list = []
    for wheel in physics_control.wheels:
        max_steer_angle_list.append(wheel.max_steer_angle)
    max_steer_angle = max(max_steer_angle_list)*np.pi/180

    previous = (0, 0)
    last_target_index = 0
    end_target_index = len(sp_x) - 5

    x_gt = []
    y_gt = []
    while True:
        print('\nRunning Step {}/{}'.format(last_target_index, end_target_index))
        world.tick()
        spectator.set_transform(dummy.get_transform())
        speed, x, y, yaw = get_state(ego, odom_tf)
        x_gt.append(x)
        y_gt.append(y)

        throttle, previous = stanley.speed_control(
            speed, desired_speed, 1/FPS, previous)
        # Carla works in LHCS  
        delta, last_target_index = stanley.stanley_control(
            sp_x[1:], -sp_y[1:], -sp_yaw[1:], x, -y, -yaw, speed, last_target_index)

        steer = delta/max_steer_angle
        steer = steer * 1  # Scaling factor

        print('Delta: ', delta)
        print('Max Steer: ', max_steer_angle)
        print('Steer: ', steer)

        ego.apply_control(carla.VehicleControl(
            throttle=throttle, steer=steer, brake=0))

        if last_target_index >= end_target_index:
            break
    
    y_gt, x_gt = np.array(y_gt), np.array(x_gt)
    print('Done!')
    plt.axis('equal')
    plt.plot(-sp_y[:end_target_index], sp_x[:end_target_index], 'g')
    plt.plot(-y_gt, x_gt, 'r')
    plt.show()


if __name__ == '__main__':
    main()
