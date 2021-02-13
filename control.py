from teleop import Keyboard
import numpy as np
import carla

# TODO: Break, gear
# control[throttle, steer, break]
def action_w(control):
    if control[0] < 1: control[0] += 0.05
    return control


def action_a(control):
    if control[1] > -1: control[1] -= 0.2
    return control


def action_s(control):
    if control[0] > -1: control[0] -= 0.05
    return control


def action_d(control):
    if control[1] < 1: control[1] += 0.2
    return control


ACTION_KEYS = {
    'w': action_w,
    'a': action_a,
    's': action_s,
    'd': action_d,
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
            ACTION_KEYS[key](control)
        elif (key == 'h'):
            control = np.array([0.0, 0.0, 0.0])
        elif (key == '\x03'):
            print('Control C')
            break
        else:
            control[1] = 0
        print(control)
        ego_vehicle.apply_control(carla.VehicleControl(throttle=control[0], steer=control[1], brake=control[2]))


if __name__=='__main__':
    main()