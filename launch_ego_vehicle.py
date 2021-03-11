""" Launces the ego vehicle.

    This script launches the ego-vechile (currently a Tesla Model 3) in Town-02. It returns an ego
    vehicle ID to be used while controlling, using control.py.

    USAGE:
        python3 launch_ego_vehicle.py [--host] [--port] [--save_lidar_data]
"""
import glob
import os
import sys
import time
import carla
import argparse
import logging
import random
import numpy as np
import shutil
from teleop import Keyboard
# from custom_locations import spawn_points_custom

from pathlib import Path

def process_point_cloud(args, point_cloud_carla, save_lidar_data):
    if save_lidar_data:
        point_cloud_carla.save_to_disk(args.data_dir + '/%.6d.ply' % point_cloud_carla.frame)
    
    # Creating a numpy array as well. To be used later    
    pcd = np.copy(np.frombuffer(point_cloud_carla.raw_data, dtype=np.dtype('float32')))
    pcd = np.reshape(pcd, (int(pcd.shape[0] / 4), 4))

def dummy_function(image):
    pass

def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '--data_dir',
        metavar='D',
        default='lidar_output',
        help='Directory to save lidar data')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--save_lidar_data', 
        default=False, 
        action='store_true',
        help='To save lidar points or not'        )
    args = argparser.parse_args()

    shutil.rmtree(args.data_dir, ignore_errors=True) 
    Path(args.data_dir).mkdir(parents=True, exist_ok=True)

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    keyboard = Keyboard(0.05)
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    client.load_world('Town02')

    
    # Setting synchronous mode
    # This is essential for proper workiong of sensors
    world = client.get_world()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05  # FPS = 1/0.05 = 20
    world.apply_settings(settings)

    try:

        world = client.get_world()
        ego_vehicle = None
        ego_cam = None
        ego_col = None
        ego_lane = None
        ego_obs = None
        ego_gnss = None
        ego_imu = None

        # --------------
        # Start recording
        # --------------
        """
        client.start_recorder('~/tutorial/recorder/recording01.log')
        """

        # --------------
        # Spawn ego vehicle
        # --------------
        # vehicles_list = custom_spawn(world)


        ego_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
        ego_bp.set_attribute('role_name','ego')
        print('\nEgo role_name is set')
        ego_color = random.choice(ego_bp.get_attribute('color').recommended_values)
        ego_bp.set_attribute('color',ego_color)
        print('\nEgo color is set')

        spawn_points = world.get_map().get_spawn_points()
        number_of_spawn_points = len(spawn_points)

        # Near a cross road
        # ego_transform = carla.Transform(carla.Location(x=-78.116066, y=-81.958496, z=-0.696164), 
        #                                carla.Rotation(pitch=1.174273, yaw=-90.156158, roll=0.000019))

        # Transform(Location(x=166.122238, y=106.114136, z=0.221694), Rotation(pitch=0.000000, yaw=-177.648560, roll=0.000014))


        ego_transform = carla.Transform(carla.Location(x=166.122238, y=106.114136, z=0.821694), 
            carla.Rotation(pitch=0.000000, yaw=-177.648560, roll=0.000014))

        ego_vehicle = world.spawn_actor(ego_bp, ego_transform)

       
        # --------------
        # Add a new LIDAR sensor to my ego
        # --------------

        # Default
        # lidar_cam = None
        # lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        # lidar_bp.set_attribute('channels',str(32))
        # lidar_bp.set_attribute('points_per_second',str(90000))
        # lidar_bp.set_attribute('rotation_frequency',str(40))
        # lidar_bp.set_attribute('range',str(20))
        # lidar_location = carla.Location(0,0,2)
        # lidar_rotation = carla.Rotation(0,0,0)
        # lidar_transform = carla.Transform(lidar_location,lidar_rotation)
        # lidar_sen = world.spawn_actor(lidar_bp,lidar_transform,attach_to=ego_vehicle)
        # lidar_sen.listen(lambda point_cloud: process_point_cloud(point_cloud, args.save_lidar_data))

        # VLP 16
        lidar_cam = None
        lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels',str(16))
        lidar_bp.set_attribute('rotation_frequency',str(20)) # Set the fps of simulator same as this
        lidar_bp.set_attribute('range',str(20))
        lidar_bp.set_attribute('lower_fov', str(-15))
        lidar_bp.set_attribute('upper_fov', str(15))
        lidar_bp.set_attribute('points_per_second',str(300000))
        lidar_bp.set_attribute('noise_stddev',str(0.173))
        # lidar_bp.set_attribute('noise_stddev',str(0.141)) Works in this case 

        lidar_location = carla.Location(0,0,2)
        lidar_rotation = carla.Rotation(0,0,0)
        lidar_transform = carla.Transform(lidar_location,lidar_rotation)
        lidar_sen = world.spawn_actor(lidar_bp,lidar_transform,attach_to=ego_vehicle)
        lidar_sen.listen(lambda point_cloud: process_point_cloud(args, point_cloud, args.save_lidar_data))

        

        # --------------
        # Enable autopilot for ego vehicle
        # --------------
        ego_vehicle.set_autopilot(False)
        
        # --------------
        # Dummy Actor for spectator
        # --------------
        dummy_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        dummy_transform = carla.Transform(carla.Location(x=-6, z=4), carla.Rotation(pitch=10.0))
        dummy = world.spawn_actor(dummy_bp, dummy_transform, attach_to=ego_vehicle, attachment_type=carla.AttachmentType.SpringArm)
        dummy.listen(lambda image: dummy_function(image))
        
        spectator = world.get_spectator()
        spectator.set_transform(dummy.get_transform())


        control = np.array([0.0, 0.0, 0.0])



        # --------------
        # Game loop. Prevents the script from finishing.
        # --------------
        input('Press Enter to Continue:')
        count = -1
        while True:
            # This is for async mode
            # world_snapshot = world.wait_for_tick() 
            
            # In synchronous mode, the client ticks the world
            world.tick()
            
            count+= 1
            if count == 0:
                print(ego_vehicle.get_transform())
                print('Ego Vehicle ID is: ', ego_vehicle.id)
         
            spectator.set_transform(dummy.get_transform())
            
    except Exception as e:
        print(e)

    finally:
        # --------------
        # Stop recording and destroy actors
        # --------------
        client.stop_recorder()
        if ego_vehicle is not None:
            if ego_cam is not None:
                ego_cam.stop()
                ego_cam.destroy()
            if ego_col is not None:
                ego_col.stop()
                ego_col.destroy()
            if ego_lane is not None:
                ego_lane.stop()
                ego_lane.destroy()
            if ego_obs is not None:
                ego_obs.stop()
                ego_obs.destroy()
            if ego_gnss is not None:
                ego_gnss.stop()
                ego_gnss.destroy()
            if ego_imu is not None:
                ego_imu.stop()
                ego_imu.destroy()
            if lidar_sen is not None:
                lidar_sen.stop()
                lidar_sen.destroy()
            if dummy is not None:
                dummy.stop()
                dummy.destroy()
            ego_vehicle.destroy()



if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\nDone with tutorial_ego.')