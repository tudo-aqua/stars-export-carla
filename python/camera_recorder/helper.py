import numpy as np

def build_projection_matrix(width, height, fov, is_behind_camera=False):
    focal = width / (2.0 * np.tan(fov * np.pi / 360.0))
    k = np.identity(3)

    if is_behind_camera:
        k[0, 0] = k[1, 1] = -focal
    else:
        k[0, 0] = k[1, 1] = focal

    k[0, 2] = width / 2.0
    k[1, 2] = height / 2.0
    return k

def get_image_point(loc, k, world_to_camera):
    # Calculate 2D projection of 3D coordinate

    # Format the input coordinate (loc is a carla.Position object)
    point = np.array([loc.x, loc.y, loc.z, 1])
    # transform to camera coordinates
    point_camera = np.dot(world_to_camera, point)

    # New we must change from UE4's coordinate system to a "standard"
    # (x, y ,z) -> (y, -z, x)
    # and we remove the fourth component also
    point_camera = [point_camera[1], -point_camera[2], point_camera[0]]

    # now project 3D->2D using the camera matrix
    point_img = np.dot(k, point_camera)
    # normalize
    point_img[0] /= point_img[2]
    point_img[1] /= point_img[2]

    return point_img[0:2]

def point_in_canvas(pos, img_width, img_height):
    """Return true if point is in canvas"""
    return (pos[0] >= 0) and (pos[0] < img_width) and (pos[1] >= 0) and (pos[1] < img_height)
