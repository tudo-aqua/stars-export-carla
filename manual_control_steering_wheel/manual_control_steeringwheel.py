#!/usr/bin/env python

# Copyright (c) 2019 Intel Labs
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

from __future__ import print_function


import argparse
import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


import carla
import logging
import pygame

from manual_control_steering_wheel.classes.HUD import HUD
from manual_control_steering_wheel.classes.World import World
from manual_control_steering_wheel.input_controls.SteeringWheelControl import SteeringWheelControl


def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None
    carla_world = None
    is_synchronous_master = False

    try:
        client = carla.Client(host="127.0.0.1", port=2000)
        client.set_timeout(2.0)

        carla_world = client.get_world()
        traffic_manager = client.get_trafficmanager(8000)  # CARLA default TM port

        # Vehicle physics (tire/suspension) is only numerically stable when
        # stepped with a small, consistent delta time. Asynchronous mode
        # ticks with whatever the server's real frame time happens to be,
        # which causes autopilot-driven vehicles near the ego to oversteer
        # and crash. Only take over as the synchronous "tick master" if
        # nobody else already has (e.g. avoid hijacking another session).
        settings = carla_world.get_settings()
        if not settings.synchronous_mode:
            is_synchronous_master = True
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            settings.substepping = True
            settings.max_substep_delta_time = 0.01
            settings.max_substeps = 16
            carla_world.apply_settings(settings)
        traffic_manager.set_synchronous_mode(True)

        display = pygame.display.set_mode(
            size=(0, 0),
            flags=pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.FULLSCREEN,
            display=0)
        display_size = pygame.display.get_surface().get_size()

        hud = HUD(display_size[0], display_size[1])
        world = World(
            carla_world, hud, actor_filter=args.filter, role_name=args.rolename,
            camera_elevation=args.camera_elevation, camera_tilt=args.camera_tilt)
        controller = SteeringWheelControl(world, client)

        clock = pygame.time.Clock()
        while True:
            clock.tick_busy_loop(60)
            carla_world.tick()
            if controller.parse_events(clock):
                return

            world.tick(clock)
            world.render(display)
            pygame.display.flip()

    finally:
        if world is not None:
            world.destroy()

        # Revert the world (and Traffic Manager) back to asynchronous mode
        # only if we're the one who switched it, and only if the server is
        # still reachable - otherwise leaving it stuck in synchronous mode
        # would freeze any other client since nobody would be left to tick it.
        if is_synchronous_master and carla_world is not None:
            try:
                settings = carla_world.get_settings()
                settings.synchronous_mode = False
                settings.fixed_delta_seconds = None
                carla_world.apply_settings(settings)
                client.get_trafficmanager(8000).set_synchronous_mode(False)
            except RuntimeError:
                pass

        pygame.quit()


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description='CARLA Manual Control Client (Steering Wheel)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.lincoln.mkz_2017',
        help='actor filter (default: "vehicle.lincoln.mkz_2017")')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--camera-elevation',
        metavar='METERS',
        default=0.0,
        type=float,
        help='offset added to the driving camera\'s height, useful for tall vehicles like trucks (default: 0.0)')
    argparser.add_argument(
        '--camera-tilt',
        metavar='DEGREES',
        default=0.0,
        type=float,
        help='offset added to the driving camera\'s downward pitch (default: 0.0)')
    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    try:
        game_loop(args)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
