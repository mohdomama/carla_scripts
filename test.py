import carla
from carla import Transform, Location
import random

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world =  client.get_world()
    blueprint_library = world.get_blueprint_library()

    # Vehicle
    vehicle_bp = random.choice(blueprint_library.filter('vehicle.bmw.*'))

    transform = Transform(Location(x=-5, y=25, z=5))
    vehicle = world.spawn_actor(vehicle_bp, transform)
    vehicle.set_autopilot(True)

    # while True:
    #     spectator = world.get_spectator()
    #     world_snapshot = world.wait_for_tick() 
    #     spectator.set_transform(vehicle.get_transform())




if __name__=='__main__':
    main()