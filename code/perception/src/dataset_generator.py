#!/usr/bin/env python3
import threading

import carla
import argparse
import os
from random import choice
from queue import Queue
from threading import Thread

# get carla host and port from environment variables
CARLA_HOST = os.environ.get('CARLA_HOST', 'localhost')
CARLA_PORT = int(os.environ.get('CARLA_PORT', '2000'))


def destroy_actors(actors):
    for actor in actors:
        actor.destroy()


def setup_empty_world(client):
    # load Town12
    # client.load_world('Town12')
    world = client.get_world()
    world.wait_for_tick()

    # destroy all actors
    destroy_actors(world.get_actors().filter('vehicle.*'))
    destroy_actors(world.get_actors().filter('walker.*'))
    destroy_actors(world.get_actors().filter('controller.*'))

    # spawn ego vehicle
    blueprint_library = world.get_blueprint_library()
    bp = blueprint_library.filter('vehicle.*')[0]
    ego_vehicle = world.spawn_actor(bp, world.get_map().get_spawn_points()[0])
    ego_vehicle.set_autopilot(True)

    # get spectator
    spectator = world.get_spectator()
    # set spectator to follow ego vehicle with offset
    spectator.set_transform(
        carla.Transform(ego_vehicle.get_location() + carla.Location(z=50),
                        carla.Rotation(pitch=-90)))

    # create traffic manager
    traffic_manager = client.get_trafficmanager(8000)
    traffic_manager.set_global_distance_to_leading_vehicle(1.0)

    # spawn traffic pedestrians
    blueprint_library = world.get_blueprint_library()
    count = 0
    while count < 14:
        bp = choice(blueprint_library.filter('walker.pedestrian.*'))
        spawn_point = carla.Transform()
        spawn_point.location = world.get_random_location_from_navigation()
        traffic_pedestrian = world.try_spawn_actor(bp,
                                                   spawn_point)
        if traffic_pedestrian is None:
            continue

        controller_bp = blueprint_library.find('controller.ai.walker')
        ai_controller = world.try_spawn_actor(controller_bp, carla.Transform(),
                                              traffic_pedestrian)
        ai_controller.start()
        ai_controller.go_to_location(
            world.get_random_location_from_navigation())
        ai_controller.set_max_speed(1.0)

        count += 1

    # spawn traffic vehicles
    for i in range(18):
        bp = choice(blueprint_library.filter('vehicle.*'))
        traffic_vehicle = world.spawn_actor(bp,
                                            world.get_map().get_spawn_points()[
                                                i + 1])
        traffic_manager.vehicle_percentage_speed_difference(traffic_vehicle,
                                                            0.0)
        traffic_vehicle.set_autopilot(True)

    return ego_vehicle


class DatasetGenerator:
    def __init__(self, output_dir):
        self.output_dir = output_dir
        self.threads = []
        self.thread_stop_events = []
        self.cameras = []
        self.instance_cameras = []
        self.directions = ["center", "right", "back", "left"]

        self.camera_queues = {
            "center": Queue(),
            "right": Queue(),
            "back": Queue(),
            "left": Queue()
        }
        self.instance_camera_queues = {
            "center": Queue(),
            "right": Queue(),
            "back": Queue(),
            "left": Queue()
        }

    def save_image(self, image, dir):
        self.camera_queues[dir].put(image)

    def save_segmented_image(self, image, dir):
        self.instance_camera_queues[dir].put(image)

    def setup_cameras(self, world, ego_vehicle):
        # get instance segmentation camera blueprint
        instance_camera_bp = world.get_blueprint_library().find(
            'sensor.camera.instance_segmentation'
        )
        # get camera blueprint
        camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('sensor_tick', '1.0')

        # set leaderboard attributes
        camera_bp.set_attribute('lens_circle_multiplier', '0.0')
        camera_bp.set_attribute('lens_circle_falloff', '5.0')
        camera_bp.set_attribute('chromatic_aberration_intensity', '0.5')
        camera_bp.set_attribute('chromatic_aberration_offset', '0.0')

        IMAGE_WIDTH = 1280
        IMAGE_HEIGHT = 720
        # set resolution
        camera_bp.set_attribute('image_size_x', str(IMAGE_WIDTH))
        camera_bp.set_attribute('image_size_y', str(IMAGE_HEIGHT))

        instance_camera_bp.set_attribute('sensor_tick', '1.0')

        # set resolution
        instance_camera_bp.set_attribute('image_size_x', str(IMAGE_WIDTH))
        instance_camera_bp.set_attribute('image_size_y', str(IMAGE_HEIGHT))

        camera_init_transform = carla.Transform(
            carla.Location(z=1.7)
        )

        for i, direction in enumerate(self.directions):
            print("Creating camera {}".format(direction))

            camera_bp.set_attribute('role_name', direction)
            instance_camera_bp.set_attribute('role_name', direction)
            # add rotation to camera transform
            camera_init_transform.rotation.yaw = i * 90
            # create camera
            camera = world.spawn_actor(
                camera_bp,
                camera_init_transform,
                attach_to=ego_vehicle,
                attachment_type=carla.AttachmentType.Rigid
            )
            # create instance segmentation camera
            instance_camera = world.spawn_actor(
                instance_camera_bp,
                camera_init_transform,
                attach_to=ego_vehicle,
                attachment_type=carla.AttachmentType.Rigid
            )
            camera.listen(
                lambda image, dir=direction: self.save_image(image, dir))
            instance_camera.listen(
                lambda image, dir=direction: self.save_segmented_image(
                    image, dir))

            self.cameras.append(camera)
            self.instance_cameras.append(instance_camera)

    # separate thread for saving images
    def save_images_worker(self, direction, stop_event):
        image_queue = self.camera_queues[direction]
        instance_image_queue = self.instance_camera_queues[direction]
        counter = 1600
        while not stop_event.is_set():
            image = image_queue.get()
            if counter < 2500:
                image.save_to_disk(
                    '{}/rgb/{}/{}.png'.format(
                        output_dir, direction,
                        counter
                    )
                )
                instance_image = instance_image_queue.get()
                instance_image.save_to_disk(
                    '{}/instance/{}/{}.png'.format(
                        output_dir, direction,
                        counter
                    )
                )
                counter += 1

    def start_saving_images(self):
        if not self.threads:
            # start separate threads for saving images
            for direction in self.directions:
                thread_stop_event = threading.Event()
                self.thread_stop_events.append(thread_stop_event)
                t = Thread(target=self.save_images_worker,
                           args=(direction, thread_stop_event))
                self.threads.append(t)
                t.start()

    def stop_saving_images(self):
        for thread_stop_event in self.thread_stop_events:
            thread_stop_event.set()
        for t in self.threads:
            t.join()


def find_ego_vehicle(world, role_name='hero'):
    if world.get_actors().filter('vehicle.*'):
        # get ego vehicle with hero role
        for actor in world.get_actors().filter('vehicle.*'):
            if actor.attributes['role_name'] == role_name:
                return actor


def create_argparse():
    argparser = argparse.ArgumentParser(
        description='CARLA Dataset Generator')
    argparser.add_argument(
        '--output-dir',
        metavar='DIR',
        default='output',
        help='Path to the output directory')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='localhost',
        help='host of the carla server'
    )
    argparser.add_argument(
        '--port',
        metavar='P',
        default=2000,
        type=int,
        help='port of the carla server'
    )
    argparser.add_argument(
        '--use-empty-world',
        action='store_true',
        help='set up an empty world and spawn ego vehicle',
        default=False
    )
    return argparser


if __name__ == '__main__':
    towns = ["Town01", "Town02", "Town03", "Town04", "Town05", "Town06",
             "Town07", "Town10", "Town11", "Town12"]
    town = towns[2]
    argparser = create_argparse()
    args = argparser.parse_args()
    output_dir = args.output_dir
    host = args.host
    port = args.port
    use_empty_world = args.use_empty_world

    client = carla.Client(host, port)
    client.set_timeout(30)
    world = client.load_world(town)
    world.wait_for_tick()

    output_dir = os.path.join(output_dir, town[-2:])

    if use_empty_world:
        ego_vehicle = setup_empty_world(client)
    else:
        ego_vehicle = find_ego_vehicle(world)

    if not ego_vehicle:
        raise RuntimeError('No vehicle found in the world')

    dataset_generator = DatasetGenerator(output_dir)
    dataset_generator.setup_cameras(world, ego_vehicle)
    dataset_generator.start_saving_images()

    # keep script running until user presses Ctrl+C
    try:
        spectator = world.get_spectator()
        while True:
            pass
            # if use_empty_world:
            #     # attach spectator to ego vehicle
            #     spectator.set_transform(
            #     carla.Transform(
            #        ego_vehicle.get_location() + carla.Location(z=50),
            #        carla.Rotation(pitch=-90)))
    except KeyboardInterrupt:
        dataset_generator.stop_saving_images()
