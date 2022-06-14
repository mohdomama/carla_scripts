import carla
import time

def dummy_function(image):
    pass

def setup_carla(FPS):
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)
    world = client.load_world('Town03')
    for i in range(10):
        world.tick()

    
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 1.0/FPS  # FPS = 1/0.05 = 20
    world.apply_settings(settings)
    world.tick()
    time.sleep(1)
    egobp = world.get_blueprint_library().find('vehicle.mini.cooperst')
    egobp.set_attribute('role_name', 'ego')


    EGOLOC = carla.Transform(carla.Location(x=155.614532, y=199.875168, z=3.015579),
                         carla.Rotation(pitch=-1.276426, yaw=180, roll=0.004011))
    ego = world.spawn_actor(egobp, EGOLOC)

    dummy_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    # dummy_transform = carla.Transform(carla.Location(
    #     x=-1, z=31), carla.Rotation(pitch=0.0))
    dummy_transform = carla.Transform(carla.Location(
        x=-5, z=10, y = 0), carla.Rotation(pitch=20.0, yaw=0, roll=0))
    dummy = world.spawn_actor(dummy_bp, dummy_transform, attach_to=ego,
                              attachment_type=carla.AttachmentType.SpringArm)
    dummy.listen(lambda image: dummy_function(image))

    spectator = world.get_spectator()
    spectator.set_transform(dummy.get_transform())

    def setup_ticks():
        for i in range(20):
            world.tick()
            spectator.set_transform(dummy.get_transform())
            ego.apply_control(carla.VehicleControl(
                throttle=0, steer=0, brake=1))
        # Clearing Brake Control | This is Important
        ego.apply_control(carla.VehicleControl(throttle=0, steer=0, brake=0))

    setup_ticks()  # Ensures that vehicle lands to the ground

    return ego, world, spectator, dummy
