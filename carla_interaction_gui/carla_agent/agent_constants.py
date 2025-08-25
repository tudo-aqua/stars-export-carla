# agent_constants.py
# Exact Python mirror of carla::traffic_manager::constants (Constants.h)
# Names, groupings, and values match C++ so edits line up 1:1. :contentReference[oaicite:0]{index=0}

class constants:
    class Networking:
        MIN_TRY_COUNT = 20
        TM_DEFAULT_PORT = 8000
        TM_TIMEOUT = 2000  # ms

    class VehicleRemoval:
        STOPPED_VELOCITY_THRESHOLD = 0.8
        BLOCKED_TIME_THRESHOLD = 90.0
        RED_TL_BLOCKED_TIME_THRESHOLD = 180.0
        DELTA_TIME_BETWEEN_DESTRUCTIONS = 10.0

    class HybridMode:
        HYBRID_MODE_DT_FL = 0.05
        HYBRID_MODE_DT = 0.05
        INV_HYBRID_DT = 1.0 / 0.05
        PHYSICS_RADIUS = 50.0

    class SpeedThreshold:
        HIGHWAY_SPEED = 60.0 / 3.6
        AFTER_JUNCTION_MIN_SPEED = 5.0 / 3.6
        INITIAL_PERCENTAGE_SPEED_DIFFERENCE = 0.0

    class PathBufferUpdate:
        MAX_START_DISTANCE = 20.0
        MINIMUM_HORIZON_LENGTH = 15.0
        HORIZON_RATE = 2.0
        HIGH_SPEED_HORIZON_RATE = 4.0

    class WaypointSelection:
        TARGET_WAYPOINT_TIME_HORIZON = 0.5
        MIN_TARGET_WAYPOINT_DISTANCE = 3.0
        JUNCTION_LOOK_AHEAD = 5.0
        SAFE_DISTANCE_AFTER_JUNCTION = 4.0
        MIN_JUNCTION_LENGTH = 8.0
        MIN_SAFE_INTERVAL_LENGTH = 0.5 * 4.0

    class LaneChange:
        MINIMUM_LANE_CHANGE_DISTANCE = 20.0
        MAXIMUM_LANE_OBSTACLE_DISTANCE = 50.0
        MAXIMUM_LANE_OBSTACLE_CURVATURE = 0.6
        INTER_LANE_CHANGE_DISTANCE = 10.0
        MIN_WPT_DISTANCE = 5.0
        MAX_WPT_DISTANCE = 20.0
        MIN_LANE_CHANGE_SPEED = 5.0
        FIFTYPERC = 50.0

    class Collision:
        BOUNDARY_EXTENSION_MINIMUM = 2.5
        BOUNDARY_EXTENSION_RATE = 4.35
        COS_10_DEGREES = 0.9848
        OVERLAP_THRESHOLD = 0.1
        LOCKING_DISTANCE_PADDING = 4.0
        COLLISION_RADIUS_STOP = 8.0
        COLLISION_RADIUS_MIN = 20.0
        COLLISION_RADIUS_RATE = 2.65
        MAX_LOCKING_EXTENSION = 10.0
        WALKER_TIME_EXTENSION = 1.5
        SQUARE_ROOT_OF_TWO = 1.414
        VERTICAL_OVERLAP_THRESHOLD = 4.0
        EPSILON = 2.0 * (2.220446049250313e-16)  # std::numeric_limits<float>::epsilon()
        MIN_REFERENCE_DISTANCE = 0.5
        MIN_VELOCITY_COLL_RADIUS = 2.0
        VEL_EXT_FACTOR = 0.36

    class FrameMemory:
        INITIAL_SIZE = 50
        GROWTH_STEP_SIZE = 50
        INV_GROWTH_STEP_SIZE = 1.0 / 50.0

    class Map:
        INFINITE_DISTANCE = float('inf')
        MAX_GEODESIC_GRID_LENGTH = 20.0
        MAP_RESOLUTION = 5.0
        INV_MAP_RESOLUTION = 1.0 / 5.0
        MAX_WPT_DISTANCE = 5.0 / 2.0 + (5.0 * 5.0)  # matches macro use in C++
        MAX_WPT_RADIANS = 0.087  # 5 deg
        DELTA = 25.0
        Z_DELTA = 500.0
        STRAIGHT_DEG = 19.0
        MIN_LANE_WIDTH = 1.0

    class TrafficLight:
        MINIMUM_STOP_TIME = 2.0
        EXIT_JUNCTION_THRESHOLD = 0.0

    class MotionPlan:
        RELATIVE_APPROACH_SPEED = 12.0 / 3.6
        MIN_FOLLOW_LEAD_DISTANCE = 2.0
        CRITICAL_BRAKING_MARGIN = 0.2
        EPSILON_RELATIVE_SPEED = 0.001
        MAX_JUNCTION_BLOCK_DISTANCE = 1.0 * 4.0
        TWO_KM = 2000.0
        ATTEMPTS_TO_TELEPORT = 5
        LANDMARK_DETECTION_TIME = 3.5
        TL_TARGET_VELOCITY = 15.0 / 3.6
        STOP_TARGET_VELOCITY = 10.0 / 3.6
        YIELD_TARGET_VELOCITY = 10.0 / 3.6
        FRICTION = 0.6
        GRAVITY = 9.81
        PI = 3.1415927
        PERC_MAX_SLOWDOWN = 0.08
        FOLLOW_LEAD_FACTOR = 2.0

    class VehicleLight:
        SUN_ALTITUDE_DEGREES_BEFORE_DAWN = 15.0
        SUN_ALTITUDE_DEGREES_AFTER_SUNSET = 165.0
        SUN_ALTITUDE_DEGREES_JUST_AFTER_DAWN = 35.0
        SUN_ALTITUDE_DEGREES_JUST_BEFORE_SUNSET = 145.0
        HEAVY_PRECIPITATION_THRESHOLD = 80.0
        FOG_DENSITY_THRESHOLD = 20.0
        MAX_DISTANCE_LIGHT_CHECK = 225.0

    class PID:
        MAX_THROTTLE = 0.85
        MAX_BRAKE = 0.7
        MAX_STEERING = 0.8
        MAX_STEERING_DIFF = 0.15
        DT = 0.05
        INV_DT = 1.0 / 0.05
        LONGITUDIAL_PARAM = (12.0, 0.05, 0.02)
        LONGITUDIAL_HIGHWAY_PARAM = (20.0, 0.05, 0.01)
        LATERAL_PARAM = (8.0, 0.04, 0.16)
        LATERAL_HIGHWAY_PARAM = (4.0, 0.04, 0.08)

    class TrackTraffic:
        BUFFER_STEP_THROUGH = 5
        INV_BUFFER_STEP_THROUGH = 1.0 / 5.0
