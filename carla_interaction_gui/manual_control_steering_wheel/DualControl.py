import math
import time
from configparser import ConfigParser

import carla
import pygame
from carla.libcarla import Client
from pygame.locals import KMOD_CTRL
from pygame.locals import KMOD_SHIFT
from pygame.locals import K_DOWN
from pygame.locals import K_ESCAPE
from pygame.locals import K_LEFT
from pygame.locals import K_RIGHT
from pygame.locals import K_SPACE
from pygame.locals import K_UP
from pygame.locals import K_a
from pygame.locals import K_d
from pygame.locals import K_q
from pygame.locals import K_s
from pygame.locals import K_w

from carla_interaction_gui.manual_control_steering_wheel.World import World


class DualControl(object):
    def __init__(self, world : World, client: Client):
        self._autopilot_enabled = False
        self._recording = False
        self._client = client
        self._hud = world.hud
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            world.player.set_autopilot(self._autopilot_enabled)
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)

        # initialize steering wheel
        pygame.joystick.init()

        joystick_count = pygame.joystick.get_count()
        if joystick_count != 1:
            raise ValueError("Please Connect Exactly One Joystick")

        self._wheel = pygame.joystick.Joystick(0)
        self._wheel.init()

        self._parser = ConfigParser()
        self._parser.read('wheel_config.ini')
        self._steer_idx = int(self._parser.get('Fanatec', 'steering_wheel_axis'))
        self._throttle_idx = int(self._parser.get('Fanatec', 'throttle_axis'))
        self._brake_idx = int(self._parser.get('Fanatec', 'brake_axis'))
        self._btn_eye = int(self._parser.get('Fanatec', 'btn_eye'))
        self._btn_flasher_left = int(self._parser.get('Fanatec', 'btn_flasher_left'))
        self._btn_flasher_right = int(self._parser.get('Fanatec', 'btn_flasher_right'))
        self._btn_park = int(self._parser.get('Fanatec', 'btn_park'))
        self._btn_wiper = int(self._parser.get('Fanatec', 'btn_wiper'))
        self._btn_burger = int(self._parser.get('Fanatec', 'btn_burger'))
        self._btn_horn = int(self._parser.get('Fanatec', 'btn_horn'))


    def parse_events(self, world, clock) -> bool:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True

            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == self._btn_burger:
                    world.hud.toggle_info()
                elif event.button == self._btn_eye:
                    world.camera_manager.toggle_camera()
                elif event.button == self._btn_wiper:
                    world.next_weather()
                elif event.button == self._btn_park:
                    self._control.gear = 1 if self._control.reverse else -1
                elif event.button == self._btn_horn:
                    if not self._recording:
                        file = f'/home/carla/recordings/recording{time.time()}.log'
                        self._client.start_recorder(file, True)
                        self._hud.notification(f'Recording into file {file}')
                        self._recording = True
                    else:
                        self._client.stop_recorder()
                        self._hud.notification('Recording Off')
                        self._recording = False
                elif event.button == self._btn_flasher_left:
                    if world.player.get_light_state() == carla.VehicleLightState.LeftBlinker:
                        world.player.set_light_state(carla.VehicleLightState.NONE)
                    else:
                        world.player.set_light_state(carla.VehicleLightState.LeftBlinker)
                elif event.button == self._btn_flasher_right:
                    if world.player.get_light_state() == carla.VehicleLightState.RightBlinker:
                        world.player.set_light_state(carla.VehicleLightState.NONE)
                    else:
                        world.player.set_light_state(carla.VehicleLightState.RightBlinker)

        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                self._parse_vehicle_wheel()
                self._control.reverse = self._control.gear < 0
            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time())
            world.player.apply_control(self._control)

        return False

    def _parse_vehicle_keys(self, keys, milliseconds):
        self._control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        self._control.hand_brake = keys[K_SPACE]

    def _parse_vehicle_wheel(self):
        numAxes = self._wheel.get_numaxes()
        jsInputs = [float(self._wheel.get_axis(i)) for i in range(numAxes)]
        # print (jsInputs)
        jsButtons = [float(self._wheel.get_button(i)) for i in
                     range(self._wheel.get_numbuttons())]

        # Custom function to map range of inputs [1, -1] to outputs [0, 1] i.e 1 from inputs means nothing is pressed
        # For the steering, it seems fine as it is
        K1 = 1.0  # 0.55
        steerCmd = K1 * math.tan(1.1 * jsInputs[self._steer_idx])

        K2 = 1.6  # 1.6
        throttleCmd = K2 + (2.05 * math.log10(
            -0.7 * jsInputs[self._throttle_idx] + 1.4) - 1.2) / 0.92
        if throttleCmd <= 0:
            throttleCmd = 0
        elif throttleCmd > 1:
            throttleCmd = 1

        brakeCmd = 1.6 + (2.05 * math.log10(
            -0.7 * jsInputs[self._brake_idx] + 1.4) - 1.2) / 0.92
        if brakeCmd <= 0:
            brakeCmd = 0
        elif brakeCmd > 1:
            brakeCmd = 1

        self._control.steer = steerCmd
        self._control.brake = brakeCmd
        self._control.throttle = throttleCmd

        #toggle = jsButtons[self._reverse_idx]

        # self._control.hand_brake = bool(jsButtons[self._handbrake_idx])

    def _parse_walker_keys(self, keys, milliseconds):
        self._control.speed = 0.0
        if keys[K_DOWN] or keys[K_s]:
            self._control.speed = 0.0
        if keys[K_LEFT] or keys[K_a]:
            self._control.speed = .01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[K_RIGHT] or keys[K_d]:
            self._control.speed = .01
            self._rotation.yaw += 0.08 * milliseconds
        if keys[K_UP] or keys[K_w]:
            self._control.speed = 5.556 if pygame.key.get_mods() & KMOD_SHIFT else 2.778
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)