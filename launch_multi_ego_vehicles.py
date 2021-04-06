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

# Town2 - Near a cross road 
# ego_transform = carla.Transform(carla.Location(x=-78.116066, y=-81.958496, z=-0.696164), 
#                                carla.Rotation(pitch=1.174273, yaw=-90.156158, roll=0.000019))
# ego_transform = carla.Transform(carla.Location(x=166.122238, y=106.114136, z=0.821694), carla.Rotation(pitch=0.000000, yaw=-177.648560, roll=0.000014))
# ego_transform = carla.Transform(carla.Location(x=166.122238, y=106.114136, z=0.821694), carla.Rotation(pitch=0.000000, yaw=-177.648560, roll=0.000014))

# Town03 - ego + cars in front, left and right
# define as global variable to automatically obtain # of cars in the control script
# ego_transforms = [
#     carla.Transform(carla.Location(x=93.220924, y=198.343231, z=1.553675), carla.Rotation(pitch=-1.277402, yaw=-179.359268, roll=-0.017578))
#     carla.Transform(carla.Location(x=93.220924, y=195.343231, z=1.553675), carla.Rotation(pitch=-1.277402, yaw=-179.359268, roll=-0.017578)),
#     carla.Transform(carla.Location(x=93.220924, y=201.343231, z=1.553675), carla.Rotation(pitch=-1.277402, yaw=-179.359268, roll=-0.017578)),
#     carla.Transform(carla.Location(x=85.220924, y=194.343231, z=1.553675), carla.Rotation(pitch=-1.277402, yaw=-179.359268, roll=-0.017578)),
#     carla.Transform(carla.Location(x=85.220924, y=198.343231, z=1.553675), carla.Rotation(pitch=-1.277402, yaw=-179.359268, roll=-0.017578)),
#     carla.Transform(carla.Location(x=85.220924, y=202.343231, z=1.553675), carla.Rotation(pitch=-1.277402, yaw=-179.359268, roll=-0.017578)),
#     carla.Transform(carla.Location(x=100.220924, y=194.343231, z=1.553675), carla.Rotation(pitch=-1.277402, yaw=-179.359268, roll=-0.017578)),
#     carla.Transform(carla.Location(x=100.220924, y=198.343231, z=1.553675), carla.Rotation(pitch=-1.277402, yaw=-179.359268, roll=-0.017578)),
#     carla.Transform(carla.Location(x=100.220924, y=202.343231, z=1.553675), carla.Rotation(pitch=-1.277402, yaw=-179.359268, roll=-0.017578))
# ]

# ego_transforms = [
#     carla.Transform(carla.Location(x=187.220924, y=198.343231, z=3.553675), carla.Rotation(pitch=-1.277402, yaw=-179.359268, roll=-0.017578)),
#     carla.Transform(carla.Location(x=187.220924, y=195.343231, z=3.553675), carla.Rotation(pitch=-1.277402, yaw=-179.359268, roll=-0.017578)),
#     carla.Transform(carla.Location(x=187.220924, y=201.343231, z=3.553675), carla.Rotation(pitch=-1.277402, yaw=-179.359268, roll=-0.017578)),
#     carla.Transform(carla.Location(x=182.220924, y=198.343231, z=3.553675), carla.Rotation(pitch=-1.277402, yaw=-179.359268, roll=-0.017578)),
#     carla.Transform(carla.Location(x=182.220924, y=195.343231, z=3.553675), carla.Rotation(pitch=-1.277402, yaw=-179.359268, roll=-0.017578)),
#     carla.Transform(carla.Location(x=182.220924, y=201.343231, z=3.553675), carla.Rotation(pitch=-1.277402, yaw=-179.359268, roll=-0.017578)),
#     carla.Transform(carla.Location(x=192.220924, y=198.343231, z=3.553675), carla.Rotation(pitch=-1.277402, yaw=-179.359268, roll=-0.017578)),
#     carla.Transform(carla.Location(x=192.220924, y=195.343231, z=3.553675), carla.Rotation(pitch=-1.277402, yaw=-179.359268, roll=-0.017578)),
#     carla.Transform(carla.Location(x=192.220924, y=201.343231, z=3.553675), carla.Rotation(pitch=-1.277402, yaw=-179.359268, roll=-0.017578))
# ]

# ego_transforms = [
#     carla.Transform(carla.Location(x=187.220924, y=198.343231, z=3.553675), carla.Rotation(pitch=-1.277402, yaw=-179.359268, roll=-0.017578)),
#     carla.Transform(carla.Location(x=167.220924, y=195.343231, z=3.553675), carla.Rotation(pitch=-1.277402, yaw=-179.359268, roll=-0.017578)),
#     carla.Transform(carla.Location(x=167.220924, y=201.343231, z=3.553675), carla.Rotation(pitch=-1.277402, yaw=-179.359268, roll=-0.017578)),
#     carla.Transform(carla.Location(x=162.220924, y=195.343231, z=3.553675), carla.Rotation(pitch=-1.277402, yaw=-179.359268, roll=-0.017578)),
#     carla.Transform(carla.Location(x=162.220924, y=201.343231, z=3.553675), carla.Rotation(pitch=-1.277402, yaw=-179.359268, roll=-0.017578)),
#     carla.Transform(carla.Location(x=192.220924, y=195.343231, z=3.553675), carla.Rotation(pitch=-1.277402, yaw=-179.359268, roll=-0.017578)),
#     carla.Transform(carla.Location(x=192.220924, y=201.343231, z=3.553675), carla.Rotation(pitch=-1.277402, yaw=-179.359268, roll=-0.017578))
# ]


# ego_transforms =[
#     # Narrow Road Scene

#     # EGO
#     carla.Transform(carla.Location(x=-145.406204, y=105.677322, z=1.501337), carla.Rotation(pitch=0.000000, yaw=-89.725250, roll=0.000011)),

#     # Traffic
#     carla.Transform(carla.Location(x=-149.446106, y=50.746162, z=1.501142), carla.Rotation(pitch=-0.000219, yaw=-89.357307, roll=0.000015)),
#     carla.Transform(carla.Location(x=-149.511078, y=45.975445, z=1.501333), carla.Rotation(pitch=0.001079, yaw=-89.357307, roll=0.000014)),
#     carla.Transform(carla.Location(x=-149.511078, y=40.975445, z=1.501333), carla.Rotation(pitch=0.001079, yaw=-89.357307, roll=0.000014)),
#     carla.Transform(carla.Location(x=-142.446106, y=50.746162, z=1.501142), carla.Rotation(pitch=-0.000219, yaw=-89.357307, roll=0.000015)),
#     carla.Transform(carla.Location(x=-142.511078, y=45.975445, z=1.501333), carla.Rotation(pitch=0.001079, yaw=-89.357307, roll=0.000014)),
#     carla.Transform(carla.Location(x=-142.511078, y=40.975445, z=1.501333), carla.Rotation(pitch=0.001079, yaw=-89.357307, roll=0.000014)),
# ]

ego_transforms = [
    # phat road scene 

    # ego
    # carla.Transform(carla.Location(x=-78.722710, y=168.484650, z=2.501263), carla.Rotation(pitch=-0.001468, yaw=-89.411835, roll=0.004054)),
    # Close to building
    # carla.Transform(carla.Location(x=-71.104912, y=116.512695, z=1.513339), carla.Rotation(pitch=0.662153, yaw=-88.713875, roll=0.070444)),

    # solid wall
    # carla.Transform(carla.Location(x=60.692669, y=-10.527537, z=1.501337), carla.Rotation(pitch=0.000382, yaw=-178.177094, roll=0.000015)),
    carla.Transform(carla.Location(x=-22.200941, y=-11.201141, z=1.501347), carla.Rotation(pitch=-0.001311, yaw=-41.370190, roll=0.000016)),



    # traffic
    # carla.Transform(carla.Location(x=-78.722710, y=168.484650, z=1.501263), carla.Rotation(pitch=-0.001468, yaw=-89.411835, roll=0.004054)),
    # carla.Transform(carla.Location(x=-78.722710, y=168.484650, z=1.501263), carla.Rotation(pitch=-0.001468, yaw=-89.411835, roll=0.004054)),
    # carla.Transform(carla.Location(x=-78.722710, y=168.484650, z=1.501263), carla.Rotation(pitch=-0.001468, yaw=-89.411835, roll=0.004054)),
    # carla.Transform(carla.Location(x=-78.722710, y=168.484650, z=1.501263), carla.Rotation(pitch=-0.001468, yaw=-89.411835, roll=0.004054)),
    # carla.Transform(carla.Location(x=-78.722710, y=168.484650, z=1.501263), carla.Rotation(pitch=-0.001468, yaw=-89.411835, roll=0.004054)),
    # carla.Transform(carla.Location(x=-78.722710, y=168.484650, z=1.501263), carla.Rotation(pitch=-0.001468, yaw=-89.411835, roll=0.004054))
]
'''
Transform(Location(x=-149.446106, y=66.746162, z=0.001142), Rotation(pitch=-0.000219, yaw=-89.357307, roll=0.000015))
Transform(Location(x=-149.511078, y=60.975445, z=0.001333), Rotation(pitch=0.001079, yaw=-89.357307, roll=0.000014))
Transform(Location(x=-149.511078, y=54.975445, z=0.001333), Rotation(pitch=0.001079, yaw=-89.357307, roll=0.000014))


'''
def process_point_cloud(args, point_cloud_carla, save_lidar_data):
    if save_lidar_data:
        point_cloud_carla.save_to_disk(args.data_dir + '/lidar' +'/%.6d.ply' % point_cloud_carla.frame)
    
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
        default='lidar_output_def',
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
        help='To save lidar points or not')
    argparser.add_argument(
        '--save_gt', 
        default=False, 
        action='store_true',
        help='To save ground truth or not')
    argparser.add_argument(
        '--town',
        default='Town03',
        help='Spawn in Town01, Town02 or Town03'
    )
    args = argparser.parse_args()

    shutil.rmtree(args.data_dir, ignore_errors=True) 
    Path(args.data_dir + '/lidar').mkdir(parents=True, exist_ok=True)

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)
    # keyboard = Keyboard(0.05)
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    client.load_world(args.town)

    
    # Setting synchronous mode
    # This is essential for proper workiong of sensors
    world = client.get_world()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05  # FPS = 1/0.05 = 20
    world.apply_settings(settings)

    try:

        world = client.get_world()
        ego_vehicles = []
        ego_cam = None
        ego_col = None
        ego_lane = None
        ego_obs = None
        ego_gnss = None
        ego_imu = None
        dummy = None

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


        num_vehicles = len(ego_transforms)
        ego_bps = []
        for i in range(num_vehicles):
            ego_bps.append(world.get_blueprint_library().find('vehicle.tesla.model3'))
        for i in range(num_vehicles):
            ego_bps[i].set_attribute('role_name', 'ego')
        print('\nEgo role_names are set')
        for i in range(num_vehicles):
            ego_color = random.choice(ego_bps[i].get_attribute('color').recommended_values)
            ego_bps[i].set_attribute('color', ego_color)
        print('\nEgo colors are set')

        # spawn_points = world.get_map().get_spawn_points()
        # number_of_spawn_points = len(spawn_points)        
        
        # spawn num_vehicles vehicles
        for i in range(num_vehicles):
            ego_vehicles.append(world.spawn_actor(ego_bps[i], ego_transforms[i]))
       
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
        lidar_bp.set_attribute('range',str(50))
        lidar_bp.set_attribute('lower_fov', str(-15))
        lidar_bp.set_attribute('upper_fov', str(15))
        lidar_bp.set_attribute('points_per_second',str(300000))
        lidar_bp.set_attribute('dropoff_general_rate',str(0.0))

        
        # lidar_bp.set_attribute('noise_stddev',str(0.173))
        # lidar_bp.set_attribute('noise_stddev',str(0.141)) Works in this case 

        lidar_location = carla.Location(0,0,2)
        lidar_rotation = carla.Rotation(0,0,0)
        lidar_transform = carla.Transform(lidar_location,lidar_rotation)
        lidar_sen = world.spawn_actor(lidar_bp,lidar_transform,attach_to=ego_vehicles[0])
        lidar_sen.listen(lambda point_cloud: process_point_cloud(args, point_cloud, args.save_lidar_data))

        

        # --------------
        # Enable autopilot for ego vehicle
        # --------------
        for i in range(num_vehicles):
            ego_vehicles[i].set_autopilot(False) 
        
        # --------------
        # Dummy Actor for spectator
        # --------------
        dummy_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        dummy_transform = carla.Transform(carla.Location(x=-6, z=4), carla.Rotation(pitch=10.0))
        dummy = world.spawn_actor(dummy_bp, dummy_transform, attach_to=ego_vehicles[0], attachment_type=carla.AttachmentType.SpringArm)
        dummy.listen(lambda image: dummy_function(image))
        
        spectator = world.get_spectator()
        spectator.set_transform(dummy.get_transform())



        gt_array = []

        # --------------
        # Game loop. Prevents the script from finishing.
        # --------------
        count = -1
        while True:
            # This is for async mode
            # world_snapshot = world.wait_for_tick() 
            
            # In synchronous mode, the client ticks the world
            world.tick()
            
            count+= 1
            if count == 0:
                for i in range(10):
                    world.tick()  # Sometimes the vehicle is spawned at a height
                    start_tf = ego_vehicles[0].get_transform()

                
                
                start_tf = ego_vehicles[0].get_transform()
                tf_matrix = np.array(start_tf.get_matrix())
                print('tf_matrix', tf_matrix)
                start_vec = np.array([start_tf.location.x, start_tf.location.y, start_tf.location.z, 1]).reshape(-1,1)
                print('Start Vec:', start_vec)

                print('After TF:\n', np.linalg.pinv(tf_matrix) @ start_vec)

                # T_start = np.array(start_tf.get_inverse_matrix()) Can be used?
                # print(start_tf.transform(start_tf.location))
                print('Ego Vehicle IDs:')
                [print(ego_vehicles[i].id, end=' ') for i in range(num_vehicles)]
                print()
                

            # print(start_tf.transform(ego_vehicle.get_transform().location))
            spectator.set_transform(dummy.get_transform())

            if args.save_gt:
                vehicle_tf =  ego_vehicles[0].get_transform()
                vehicle_tf_loc = np.array([vehicle_tf.location.x, vehicle_tf.location.y, vehicle_tf.location.z, 1]).reshape(-1,1)
                vehicle_tf_odom =  np.linalg.pinv(tf_matrix) @ vehicle_tf_loc
                gt_array.append(vehicle_tf_odom.flatten()[:-1])

            
    except Exception as e:
        print(e)

    finally:

        if args.save_gt:
            gt_array = np.array(gt_array)
            np.savetxt( args.data_dir + '/gt.csv', gt_array, delimiter=',')

        # --------------
        # Stop recording and destroy actors
        # --------------
        client.stop_recorder()
        if ego_vehicles is not None:
            print('Here')
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
            print('Here Here')
            [ego_vehicles[i].destroy() for i in range(num_vehicles)]
            world.tick()



if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\nDone with launch_multi_ego_vehicles.')