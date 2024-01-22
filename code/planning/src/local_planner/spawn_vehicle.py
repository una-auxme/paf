import carla
import os
import rospy


CARLA_HOST = os.environ.get('CARLA_HOST', 'paf23-carla-simulator-1')
CARLA_PORT = int(os.environ.get('CARLA_PORT', '2000'))

client = carla.Client(CARLA_HOST, CARLA_PORT)

world = client.get_world()
world.wait_for_tick()


blueprint_library = world.get_blueprint_library()
# bp = blueprint_library.filter('vehicle.*')[0]
# vehicle = world.spawn_actor(bp, world.get_map().get_spawn_points()[0])
bp = blueprint_library.filter("model3")[0]
world = client.get_world()
loc = carla.Location(x=983.59, y=-5450.01, z=703.810118745277)
print("location: " + str(loc))
spawnPoint = carla.Transform(loc, carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0))
print("spawnPoint: " + str(spawnPoint))
vehicle = world.spawn_actor(bp, spawnPoint)

print("spawned vehicle: " + str(vehicle.get_location()))
vehicle.set_autopilot(False)
vehicle.set_location(loc)
print("spawned vehicle: " + str(vehicle.get_location()))

# get spectator
spectator = world.get_spectator()
# set spectator to follow ego vehicle with offset
spectator.set_transform(
    carla.Transform(loc + carla.Location(z=50),
                    carla.Rotation(pitch=-90)))
print(spectator.get_transform())
client.reload_world()
