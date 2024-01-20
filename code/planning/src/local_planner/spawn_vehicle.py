import carla
import os


CARLA_HOST = os.environ.get('CARLA_HOST', 'paf23-carla-simulator-1')
CARLA_PORT = int(os.environ.get('CARLA_PORT', '2000'))

client = carla.Client(CARLA_HOST, CARLA_PORT)

world = client.get_world()
world.wait_for_tick()


blueprint_library = world.get_blueprint_library()
bp = blueprint_library.filter('vehicle.*')[0]
vehicle = world.spawn_actor(bp, world.get_map().get_spawn_points()[0])
vehicle.set_autopilot(False)

# get spectator
spectator = world.get_spectator()
# set spectator to follow ego vehicle with offset
spectator.set_transform(
    carla.Transform(vehicle.get_location() + carla.Location(z=50),
                    carla.Rotation(pitch=-90)))
