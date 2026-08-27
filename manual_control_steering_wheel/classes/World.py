import random

import carla

from manual_control_steering_wheel.classes.CollisionSensor import CollisionSensor
from manual_control_steering_wheel.classes.LaneInvasionSensor import LaneInvasionSensor
from manual_control_steering_wheel.classes.GnssSensor import GnssSensor
from manual_control_steering_wheel.classes.CameraManager import CameraManager
from manual_control_steering_wheel.helpers import find_weather_presets, get_actor_display_name


class World(object):
    def __init__(self, carla_world, hud, actor_filter='vehicle.lincoln.mkz_2017', role_name='hero'):
        self.world = carla_world
        self.hud = hud
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.camera_manager = None
        self._actor_filter = actor_filter
        self._role_name = role_name
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self.restart()
        self.world.on_tick(hud.on_world_tick)

    def restart(self):
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        # Get a random blueprint matching the actor filter (a filter like "walker.pedestrian.*"
        # matches many blueprints; an exact id like "vehicle.lincoln.mkz_2017" matches one).
        blueprint_list = self.world.get_blueprint_library().filter(self._actor_filter)
        if not blueprint_list:
            raise ValueError(f"Couldn't find any blueprints matching filter '{self._actor_filter}'")
        blueprint = random.choice(blueprint_list)
        blueprint.set_attribute('role_name', self._role_name)
        # if blueprint.has_attribute('color'):
        #     color = random.choice(blueprint.get_attribute('color').recommended_values)
        #     blueprint.set_attribute('color', color)
        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        while self.player is None:
            spawn_points = self.world.get_map().get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        self.camera_manager.render(display)
        self.hud.render(display)

    def destroy(self):
        sensors = [
            self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.player is not None:
            self.player.destroy()