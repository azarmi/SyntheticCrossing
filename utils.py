
import cv2
import carla 
import numpy as np

# Standard RGB data processing:
def rgb_callback(image, data_dict):
    data_dict['rgb_image'] = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
    
# Semantic segmentation data processing
def sem_callback(image, data_dict):
    image.convert(carla.ColorConverter.CityScapesPalette)
    data_dict['sem_image'] = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
    
def opt_callback(data, data_dict):
    image = data.get_color_coded_flow()
    img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
    img *= 10
    img[:,:,3] = 255
    data_dict['opt_image'] = img
    
def build_projection_matrix(w, h, fov):
    focal = w / (2.0 * np.tan(fov * np.pi / 360.0))
    K = np.identity(3)
    K[0, 0] = K[1, 1] = focal
    K[0, 2] = w / 2.0
    K[1, 2] = h / 2.0
    return K

def get_screen_points(camera, K, image_w, image_h, points3d):
    
    # get 4x4 matrix to transform points from world to camera coordinates
    world_2_camera = np.array(camera.get_transform().get_inverse_matrix())

    # build the points array in numpy format as (x, y, z, 1) to be operable with a 4x4 matrix
    points_temp = []
    for p in points3d:
        points_temp += [p.x, p.y, p.z, 1]
    points = np.array(points_temp).reshape(-1, 4).T
    
    # convert world points to camera space
    points_camera = np.dot(world_2_camera, points)
    
    # New we must change from UE4's coordinate system to an "standard"
    # (x, y ,z) -> (y, -z, x)
    # and we remove the fourth component also
    points = np.array([
        points_camera[1],
        points_camera[2] * -1,
        points_camera[0]])
    
    # Finally we can use our K matrix to do the actual 3D -> 2D.
    points_2d = np.dot(K, points)

    # normalize the values and transpose
    points_2d = np.array([
        points_2d[0, :] / points_2d[2, :],
        points_2d[1, :] / points_2d[2, :],
        points_2d[2, :]]).T

    return points_2d


def draw_points_on_buffer(buffer, image_w, image_h, points_2d, color, size=4):
    half = int(size / 2)
    # draw each point
    for p in points_2d:
        x = int(p[0])
        y = int(p[1])
        for j in range(y - half, y + half):
            if (j >=0 and j <image_h):
                for i in range(x - half, x + half):
                    if (i >=0 and i <image_w):
                        buffer[j][i][0] = color[0]
                        buffer[j][i][1] = color[1]
                        buffer[j][i][2] = color[2]

def draw_line_on_buffer(buffer, image_w, image_h, points_2d, color, size=4):
    x0 = int(points_2d[0][0])
    y0 = int(points_2d[0][1])
    x1 = int(points_2d[1][0])
    y1 = int(points_2d[1][1])
    dx = abs(x1 - x0)
    if x0 < x1:
        sx = 1
    else:
        sx = -1
    dy = -abs(y1 - y0)
    if y0 < y1:
        sy = 1
    else:
        sy = -1
    err = dx + dy
    while True:
        draw_points_on_buffer(buffer, image_w, image_h, ((x0,y0),), color, size)
        if (x0 == x1 and y0 == y1):
            break
        e2 = 2 * err
        if (e2 >= dy):
            err += dy
            x0 += sx
        if (e2 <= dx):
            err += dx
            y0 += sy

def draw_skeleton(buffer, image_w, image_h, boneIndex, points2d, color, size=4):
    # draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_root"]], points2d[boneIndex["crl_hips__C"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_hips__C"]], points2d[boneIndex["crl_spine__C"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_hips__C"]], points2d[boneIndex["crl_thigh__R"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_hips__C"]], points2d[boneIndex["crl_thigh__L"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_spine__C"]], points2d[boneIndex["crl_spine01__C"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_spine01__C"]], points2d[boneIndex["crl_shoulder__L"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_spine01__C"]], points2d[boneIndex["crl_neck__C"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_spine01__C"]], points2d[boneIndex["crl_shoulder__R"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_shoulder__L"]], points2d[boneIndex["crl_arm__L"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_arm__L"]], points2d[boneIndex["crl_foreArm__L"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_foreArm__L"]], points2d[boneIndex["crl_hand__L"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_hand__L"]], points2d[boneIndex["crl_handThumb__L"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_hand__L"]], points2d[boneIndex["crl_handIndex__L"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_hand__L"]], points2d[boneIndex["crl_handMiddle__L"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_hand__L"]], points2d[boneIndex["crl_handRing__L"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_hand__L"]], points2d[boneIndex["crl_handPinky__L"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_handThumb__L"]], points2d[boneIndex["crl_handThumb01__L"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_handThumb01__L"]], points2d[boneIndex["crl_handThumb02__L"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_handThumb02__L"]], points2d[boneIndex["crl_handThumbEnd__L"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_handIndex__L"]], points2d[boneIndex["crl_handIndex01__L"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_handIndex01__L"]], points2d[boneIndex["crl_handIndex02__L"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_handIndex02__L"]], points2d[boneIndex["crl_handIndexEnd__L"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_handMiddle__L"]], points2d[boneIndex["crl_handMiddle01__L"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_handMiddle01__L"]], points2d[boneIndex["crl_handMiddle02__L"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_handMiddle02__L"]], points2d[boneIndex["crl_handMiddleEnd__L"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_handRing__L"]], points2d[boneIndex["crl_handRing01__L"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_handRing01__L"]], points2d[boneIndex["crl_handRing02__L"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_handRing02__L"]], points2d[boneIndex["crl_handRingEnd__L"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_handPinky__L"]], points2d[boneIndex["crl_handPinky01__L"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_handPinky01__L"]], points2d[boneIndex["crl_handPinky02__L"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_handPinky02__L"]], points2d[boneIndex["crl_handPinkyEnd__L"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_neck__C"]], points2d[boneIndex["crl_Head__C"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_Head__C"]], points2d[boneIndex["crl_eye__L"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_Head__C"]], points2d[boneIndex["crl_eye__R"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_shoulder__R"]], points2d[boneIndex["crl_arm__R"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_arm__R"]], points2d[boneIndex["crl_foreArm__R"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_foreArm__R"]], points2d[boneIndex["crl_hand__R"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_hand__R"]], points2d[boneIndex["crl_handThumb__R"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_hand__R"]], points2d[boneIndex["crl_handIndex__R"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_hand__R"]], points2d[boneIndex["crl_handMiddle__R"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_hand__R"]], points2d[boneIndex["crl_handRing__R"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_hand__R"]], points2d[boneIndex["crl_handPinky__R"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_handThumb__R"]], points2d[boneIndex["crl_handThumb01__R"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_handThumb01__R"]], points2d[boneIndex["crl_handThumb02__R"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_handThumb02__R"]], points2d[boneIndex["crl_handThumbEnd__R"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_handIndex__R"]], points2d[boneIndex["crl_handIndex01__R"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_handIndex01__R"]], points2d[boneIndex["crl_handIndex02__R"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_handIndex02__R"]], points2d[boneIndex["crl_handIndexEnd__R"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_handMiddle__R"]], points2d[boneIndex["crl_handMiddle01__R"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_handMiddle01__R"]], points2d[boneIndex["crl_handMiddle02__R"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_handMiddle02__R"]], points2d[boneIndex["crl_handMiddleEnd__R"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_handRing__R"]], points2d[boneIndex["crl_handRing01__R"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_handRing01__R"]], points2d[boneIndex["crl_handRing02__R"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_handRing02__R"]], points2d[boneIndex["crl_handRingEnd__R"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_handPinky__R"]], points2d[boneIndex["crl_handPinky01__R"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_handPinky01__R"]], points2d[boneIndex["crl_handPinky02__R"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_handPinky02__R"]], points2d[boneIndex["crl_handPinkyEnd__R"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_thigh__R"]], points2d[boneIndex["crl_leg__R"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_leg__R"]], points2d[boneIndex["crl_foot__R"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_foot__R"]], points2d[boneIndex["crl_toe__R"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_toe__R"]], points2d[boneIndex["crl_toeEnd__R"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_thigh__L"]], points2d[boneIndex["crl_leg__L"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_leg__L"]], points2d[boneIndex["crl_foot__L"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_foot__L"]], points2d[boneIndex["crl_toe__L"]]), color, size)
    draw_line_on_buffer(buffer, image_w, image_h, (points2d[boneIndex["crl_toe__L"]], points2d[boneIndex["crl_toeEnd__L"]]), color, size)
    return buffer

def get_image_point(loc, K, w2c):
    # Calculate 2D projection of 3D coordinate

    # Format the input coordinate (loc is a carla.Position object)
    point = np.array([loc.x, loc.y, loc.z, 1])
    # transform to camera coordinates
    point_camera = np.dot(w2c, point)

    # New we must change from UE4's coordinate system to an "standard"
    # (x, y ,z) -> (y, -z, x)
    # and we remove the fourth componebonent also
    point_camera = [point_camera[1], -point_camera[2], point_camera[0]]

    # now project 3D->2D using the camera matrix
    point_img = np.dot(K, point_camera)
    # normalize
    point_img[0] /= point_img[2]
    point_img[1] /= point_img[2]

    return point_img[0:2]



def draw_pose(image, ped, camera, K, image_w, image_h):
    bones = ped.get_bones()
    boneIndex = {}  
    points = []
    for i, bone in enumerate(bones.bone_transforms):
        boneIndex[bone.name] = i
        points.append(bone.world.location)

    # project the 3d points to 2d screen
    points2d = get_screen_points(camera, K, image_w, image_h, points)

    # draw the skeleton lines
    sktn_img = draw_skeleton(image, image_w, image_h, boneIndex, points2d, (0, 255, 0), 2)

    # draw the bone points
    draw_points_on_buffer(sktn_img, image_w, image_h, points2d[1:], (255, 0, 0), 2)

    return sktn_img

    
def draw_bbox(img, world, vehicle, camera, K ):
    for npc in world.get_actors().filter('*pedestrian*'):

        bb = npc.bounding_box
        dist = npc.get_transform().location.distance(vehicle.get_transform().location)

        # Filter for the vehicles within 50m
        if dist < 50:

        # Calculate the dot product between the forward vector
        # of the vehicle and the vector between the vehicle
        # and the other vehicle. We threshold this dot product
        # to limit to drawing bounding boxes IN FRONT OF THE CAMERA
            forward_vec = vehicle.get_transform().get_forward_vector()
            ray = npc.get_transform().location - vehicle.get_transform().location

            if forward_vec.dot(ray) > 1:
                world_2_camera = np.array(camera.get_transform().get_inverse_matrix())
                p1 = get_image_point(bb.location, K, world_2_camera)
                verts = [v for v in bb.get_world_vertices(npc.get_transform())]

                ''' Vehicles 2D Box '''
                x_max = -10000
                x_min = 10000
                y_max = -10000
                y_min = 10000

                for vert in verts:
                    p = get_image_point(vert, K, world_2_camera)
                    # Find the rightmost vertex
                    if p[0] > x_max:
                        x_max = p[0]
                    # Find the leftmost vertex
                    if p[0] < x_min:
                        x_min = p[0]
                    # Find the highest vertex
                    if p[1] > y_max:
                        y_max = p[1]
                    # Find the lowest  vertex
                    if p[1] < y_min:
                        y_min = p[1]

                cv2.line(img, (int(x_min),int(y_min)), (int(x_max),int(y_min)), (0,0,255, 255), 1)
                cv2.line(img, (int(x_min),int(y_max)), (int(x_max),int(y_max)), (0,0,255, 255), 1)
                cv2.line(img, (int(x_min),int(y_min)), (int(x_min),int(y_max)), (0,0,255, 255), 1)
                cv2.line(img, (int(x_max),int(y_min)), (int(x_max),int(y_max)), (0,0,255, 255), 1)
    return img
