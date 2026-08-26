#!/usr/bin/env python

# Copyright (c) 2019 Intel Labs
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

from __future__ import print_function


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

from World import World
from DualControl import DualControl
from HUD import HUD


def game_loop():
    pygame.init()
    pygame.font.init()
    world = None

    try:
        client = carla.Client(host="127.0.0.1", port=2000)
        client.set_timeout(2.0)

        display = pygame.display.set_mode(
            size=(0, 0),
            flags=pygame.HWSURFACE | pygame.DOUBLEBUF | pygame.FULLSCREEN,
            display=0)
        display_size = pygame.display.get_surface().get_size()

        hud = HUD(display_size[0], display_size[1])
        world = World(client.get_world(), hud)
        controller = DualControl(world)

        clock = pygame.time.Clock()
        while True:
            clock.tick_busy_loop(60)
            if controller.parse_events(world, clock):
                return

            world.tick(clock)
            world.render(display)
            pygame.display.flip()

    finally:
        if world is not None:
            world.destroy()

        pygame.quit()


if __name__ == '__main__':
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    try:
        game_loop()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
