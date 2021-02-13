from teleop import Keyboard
import numpy as np
import carla

ACTION_KEYS = {
    'w': np.array([0.05,     0, 0   ]),
    'a': np.array([0.00, -0.05, 0   ]),
    's': np.array([0.00,     0, 0.05]),
    'd': np.array([0.00,  0.05, 0   ]),
}


def main():
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    keyboard = Keyboard(0.05)
    control = np.array([0.0, 0.0, 0.0])

    ego_id = int(input('Enter Ego Vehicle ID:\n'))

    actor_list = world.get_actors()
    print('Actor List: ', actor_list)
    
    ego_vehicle = actor_list.find(ego_id)

    while True:
        key = keyboard.read()
        if key in ACTION_KEYS:
            control += ACTION_KEYS[key]
            control = np.clip(control, -1, 1)
        elif (key == 'h'):
            control = np.array([0.0, 0.0, 0.0])
        elif (key == '\x03'):
            print('Control C')
            break
            
        else:
            pass
        print(control)
        ego_vehicle.apply_control(carla.VehicleControl(throttle=control[0], steer=control[1], brake=control[2]))


if __name__=='__main__':
    main()