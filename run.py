import os
import carla 
import math 
import random 
import time 
import numpy as np
import cv2
import subprocess
from multiprocessing import Process
import psutil
from utils import *

PATH_OF_CARLA = r'C:\CARLA_0.9.13\WindowsNoEditor\CarlaUE4.exe'
def CARLA(): subprocess.call(PATH_OF_CARLA)


if __name__ == '__main__':
    FPS = 30
    Process(target=CARLA, args=()).start() # Open Carla in a subprocess
    
    for t in range(10): print('>', end=''); time.sleep(1) # Wait 10 seconds for Carla to launched
    print(' Started.')
    

    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()
    print('>>>>')

    # world.unload_map_layer(carla.MapLayer.Buildings)
    # world.unload_map_layer(carla.MapLayer.Foliage)
    # world.unload_map_layer(carla.MapLayer.Decals)
    # world.unload_map_layer(carla.MapLayer.ParkedVehicles)
    # world.unload_map_layer(carla.MapLayer.Props)
    # world.unload_map_layer(carla.MapLayer.Walls)
    # world.unload_map_layer(carla.MapLayer.Particles)
    # world.unload_map_layer(carla.MapLayer.StreetLights)
    # world.wait_for_tick()

    # Get list of map layers
    map_layers = [
        # carla.MapLayer.Buildings,
        carla.MapLayer.Foliage,
        carla.MapLayer.Decals,
        carla.MapLayer.ParkedVehicles,
        carla.MapLayer.Props,
        carla.MapLayer.Walls,
        carla.MapLayer.Particles,
        carla.MapLayer.StreetLights
    ]

    spectator = world.get_spectator()
    transform = carla.Transform(carla.Location(x=-44.389740, y=64.869057, z=15.139067), carla.Rotation(pitch=-37.372162, yaw=-90.537979, roll=0.000165))
    spectator.set_transform(transform)
    world.wait_for_tick()

    settings = client.get_world().get_settings()
    settings.fixed_delta_seconds = 1.0 / FPS  # Set the simulation timestep to match 30 FPS
    client.get_world().apply_settings(settings)

    spawn_points = world.get_map().get_spawn_points()
    vehicle_bps = bp_lib.find('vehicle.audi.tt')
    tran = carla.Transform()

    # vehicle = world.try_spawn_actor(vehicle_bps, spawn_points[137])
    # tran.location = carla.Location(vehicle.get_location().x, vehicle.get_location().y-8, vehicle.get_location().z)
    # tran.rotation = vehicle.get_transform().rotation
    # vehicle.set_transform(tran)

    vehicle = world.try_spawn_actor(vehicle_bps, spawn_points[25])
    
    # vehicle = world.try_spawn_actor(vehicle_bps, spawn_points[35])
    # tran.location = carla.Location(vehicle.get_location().x, vehicle.get_location().y, vehicle.get_location().z)
    # tran.rotation = vehicle.get_transform().rotation
    # vehicle.set_transform(tran)

    # vehicle = world.try_spawn_actor(vehicle_bps, spawn_points[37])
    # tran.location = carla.Location(vehicle.get_location().x, vehicle.get_location().y, vehicle.get_location().z)
    # tran.rotation = vehicle.get_transform().rotation
    # vehicle.set_transform(tran)

    # vehicle = world.try_spawn_actor(vehicle_bps, spawn_points[87])
    # tran.location = carla.Location(vehicle.get_location().x, vehicle.get_location().y, vehicle.get_location().z)
    # tran.rotation = vehicle.get_transform().rotation
    # vehicle.set_transform(tran)

    # vehicle = world.try_spawn_actor(vehicle_bps, spawn_points[21])
    # tran.location = carla.Location(vehicle.get_location().x, vehicle.get_location().y, vehicle.get_location().z)
    # tran.rotation = vehicle.get_transform().rotation
    # vehicle.set_transform(tran)

    # vehicle = world.try_spawn_actor(vehicle_bps, spawn_points[102])
    transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x=-5,z=3)), vehicle.get_transform().rotation) 
    spectator.set_transform(transform)
    

    world.wait_for_tick()

    ''' other road users'''
    my_vehicles = []
    for i in range(100): 
        vehicle_bp = random.choice(bp_lib.filter('vehicle')) 
        new_vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))    
        world.wait_for_tick()
        if new_vehicle is not None:
            new_vehicle.set_autopilot(True)
        if not new_vehicle is None:
            my_vehicles.append(new_vehicle)

    ''''''

    # camera_init_trans = carla.Transform(carla.Location(z=2))

    # camera_bp = bp_lib.find('sensor.camera.rgb') 
    # camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)

    # sem_camera_bp = bp_lib.find('sensor.camera.semantic_segmentation') 
    # sem_camera = world.spawn_actor(sem_camera_bp, camera_init_trans, attach_to=vehicle)

    # opt_camera_bp = bp_lib.find('sensor.camera.optical_flow') 
    # opt_camera = world.spawn_actor(opt_camera_bp, camera_init_trans, attach_to=vehicle)


    # Define the desired framerate

    # Get the blueprint library
    bp_lib = world.get_blueprint_library()

    # Set up initial transform for the sensors
    camera_init_trans = carla.Transform(carla.Location(z=2))

    # Find RGB camera blueprint
    camera_bp = bp_lib.find('sensor.camera.rgb')
    # Set the framerate
    camera_bp.set_attribute('sensor_tick', str(1.0 / FPS))
    # Spawn the RGB camera sensor
    camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)

    # Find semantic segmentation camera blueprint
    sem_camera_bp = bp_lib.find('sensor.camera.semantic_segmentation')
    # Set the framerate
    sem_camera_bp.set_attribute('sensor_tick', str(1.0 / FPS))
    # Spawn the semantic segmentation camera sensor
    sem_camera = world.spawn_actor(sem_camera_bp, camera_init_trans, attach_to=vehicle)

    # Find optical flow camera blueprint
    opt_camera_bp = bp_lib.find('sensor.camera.optical_flow')
    # Set the framerate
    opt_camera_bp.set_attribute('sensor_tick', str(1.0 / FPS))
    # Spawn the optical flow camera sensor
    opt_camera = world.spawn_actor(opt_camera_bp, camera_init_trans, attach_to=vehicle)


    image_w = camera_bp.get_attribute("image_size_x").as_int()
    image_h = camera_bp.get_attribute("image_size_y").as_int()
    fov = camera_bp.get_attribute("fov").as_float()

    sensor_data = {'rgb_image': np.zeros((image_h, image_w, 4)),
                'sem_image': np.zeros((image_h, image_w, 4)),
                'opt_image': np.zeros((image_h, image_w, 4))}

    cv2.namedWindow('All cameras', cv2.WINDOW_AUTOSIZE)

    # Tile all data in one array
    tiled = np.concatenate((sensor_data['rgb_image'], sensor_data['sem_image'], sensor_data['opt_image']), axis=0)

    # Display with imshow
    cv2.imshow('All cameras',tiled)
    cv2.waitKey(1)

    # Set sensors recording
    camera.listen(lambda image: rgb_callback(image, sensor_data))
    sem_camera.listen(lambda image: sem_callback(image, sensor_data))
    opt_camera.listen(lambda image: opt_callback(image, sensor_data))

    K = build_projection_matrix(image_w, image_h, fov)

    walker_bpX = bp_lib.filter("walker.pedestrian.*")
    control = carla.WalkerControl()
    control.speed = 2
    # control.jump = 0
    i = 0
    new_pedest = True






    number_of_walkers = 300
    walkers_list = []
    all_id = []

    SpawnActor = carla.command.SpawnActor
    percentagePedestriansRunning = 0.0      # how many pedestrians will run
    percentagePedestriansCrossing = 0.0     # how many pedestrians will walk through the road

    # 1. take all the random locations to spawn
    spawn_points = []
    for i in range(number_of_walkers):
        spawn_point = carla.Transform()
        loc = world.get_random_location_from_navigation()
        if (loc != None):
            spawn_point.location = loc
            spawn_points.append(spawn_point)
    # 2. we spawn the walker object  
    batch = []
    walker_speed = []
    for spawn_point in spawn_points:
        walker_bp = random.choice(bp_lib.filter('walker.pedestrian.*'))
        # set as not invincible
        if walker_bp.has_attribute('is_invincible'):
            walker_bp.set_attribute('is_invincible', 'false')
        # set the max speed
        if walker_bp.has_attribute('speed'):
            if (random.random() > percentagePedestriansRunning):
                # walking
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
            else:
                # running
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
        else:
            print("Walker has no speed")
            walker_speed.append(0.0)
        batch.append(SpawnActor(walker_bp, spawn_point))
    results = client.apply_batch_sync(batch, True)

    walker_speed2 = []
    for i in range(len(results)):
        if not results[i].error:
            walkers_list.append({"id": results[i].actor_id})
            walker_speed2.append(walker_speed[i])
    walker_speed = walker_speed2

    # 3. we spawn the walker controller
    batch = []
    walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
    for i in range(len(walkers_list)):
        batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
    results = client.apply_batch_sync(batch, True)
    for i in range(len(results)):
        if not results[i].error:
            walkers_list[i]["con"] = results[i].actor_id
    # 4. we put together the walkers and controllers id to get the objects from their id
    for i in range(len(walkers_list)):
        all_id.append(walkers_list[i]["con"])
        all_id.append(walkers_list[i]["id"])
    all_actors = world.get_actors(all_id)

    world.wait_for_tick()

    # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
    # set how many pedestrians can cross the road
    world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
    for i in range(0, len(all_id), 2):
        # start walker
        all_actors[i].start()
        # set walk to random point
        all_actors[i].go_to_location(world.get_random_location_from_navigation())
        # max speed
        all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))









    # Weather settings
    weather_conditions = [
        carla.WeatherParameters.ClearNoon,
        carla.WeatherParameters.CloudyNoon,
        carla.WeatherParameters.WetNoon,
        carla.WeatherParameters.WetCloudyNoon,
        carla.WeatherParameters.MidRainyNoon,
        carla.WeatherParameters.HardRainNoon,
        carla.WeatherParameters.SoftRainNoon,
    ]

    last_layer_change_time = time.time()
    last_weather_change_time = time.time()
    last_day_time_change_time = time.time()

    ROOT= os. getcwd()
    OUT_PATH = f'{ROOT}\data'
    print(OUT_PATH)

    if not os.path.exists(OUT_PATH): os.mkdir(OUT_PATH)
    videos = {}
    for stream in sensor_data:
        videos[stream] = cv2.VideoWriter(f'{OUT_PATH}\carla_{stream}.avi',cv2.VideoWriter_fourcc(*'DIVX'), FPS//3, (image_w, image_h))
    videos['pose'] = cv2.VideoWriter(f'{OUT_PATH}\carla_pose.avi',cv2.VideoWriter_fourcc(*'DIVX'), FPS//3, (image_w, image_h))
    videos['box'] = cv2.VideoWriter(f'{OUT_PATH}\carla_box.avi',cv2.VideoWriter_fourcc(*'DIVX'), FPS//3, (image_w, image_h))

    while True:
        current_time = time.time()
        time_difference_layer = current_time - last_layer_change_time
        time_difference_weather = current_time - last_weather_change_time
        time_difference_daytime = current_time - last_day_time_change_time

        if time_difference_layer >= 2:
            # Randomly select map layers to load and unload
            layer_to_load = random.choice(map_layers)
            layer_to_unload = random.choice(map_layers)
            
            # Unload selected layer
            world.unload_map_layer(layer_to_unload)
            
            # Load selected layer
            world.load_map_layer(layer_to_load)

            last_layer_change_time = current_time
        

        if time_difference_weather >= 2:
            # Change weather and sun altitude angle together
            weather = random.choice(weather_conditions)
            sun_altitude_angle = random.uniform(-90, 90)
            weather_params = carla.WeatherParameters(
                cloudiness=weather.cloudiness,
                precipitation=weather.precipitation,
                precipitation_deposits=weather.precipitation_deposits,
                wind_intensity=weather.wind_intensity,
                sun_azimuth_angle=weather.sun_azimuth_angle,
                sun_altitude_angle=sun_altitude_angle,
                fog_density=weather.fog_density,
                fog_distance=weather.fog_distance,
                fog_falloff=weather.fog_falloff,
                wetness=weather.wetness,
                scattering_intensity=weather.scattering_intensity,
                mie_scattering_scale=weather.mie_scattering_scale,
                rayleigh_scattering_scale=weather.rayleigh_scattering_scale
            )
            world.set_weather(weather_params)
            last_weather_change_time = current_time

        # if time_difference_weather >= 1:
        #     # Change weather every second
        #     weather = random.choice(weather_conditions)
        #     world.set_weather(weather)
        #     last_weather_change_time = current_time
        
        # if time_difference_daytime >= 1:
        #     # Change day time every second
        #     sun_altitude_angle = random.uniform(0, 90)
        #     weather_params = carla.WeatherParameters(sun_altitude_angle=sun_altitude_angle)
        #     world.set_weather(weather_params)
        #     last_day_time_change_time = current_time


        # vehicle.apply_control(carla.VehicleControl(throttle=.1))

        if new_pedest:
            trans = carla.Transform(carla.Location(x=-35, y=37.5, z=1), carla.Rotation(yaw=270))
            ped_bp = random.choice(walker_bpX)
            ped = world.try_spawn_actor(ped_bp, trans)
            world.wait_for_tick()
            crossing = random.choice([True, True])
            
        if i < 90:
            if crossing: 
                control.direction.y = 0
                control.direction.x = -1
                control.direction.z = 0
            else:
                control.direction.y = 0
                control.direction.x = 1
                control.direction.z = 0
            new_pedest = False

        ped.apply_control(control)
        time.sleep(0.1)
        i += 1
        

        for stream in sensor_data:
            videos[stream].write(sensor_data[stream][:,:,:3])
        # posex = draw_pose(sensor_data['rgb_image'].copy(), ped, camera, K, image_w, image_h)
        # videos['pose'].write(posex[:,:,:3])
        # boxx = draw_bbox(sensor_data['rgb_image'].copy(), world, vehicle, camera, K)
        # videos['box'].write(posex[:,:,:3])

        tiled = np.concatenate((sensor_data['rgb_image'], sensor_data['sem_image'], sensor_data['opt_image']), axis=0)
        cv2.imshow('All cameras', cv2.resize(tiled, (tiled.shape[1]//2, tiled.shape[0]//2)))
        # cv2.imshow('Skeleton', posex)
        # cv2.imshow('Bounding Box', boxx)

        if i > 90:
            ped.destroy()
            new_pedest = True
            i = 0
            
        if cv2.waitKey(1) == ord('q'):
            break

    # Stop sensors and destroy OpenCV window
    camera.stop()
    sem_camera.stop()
    opt_camera.stop()

    cv2.destroyAllWindows()

    for stream in sensor_data:
        videos[stream].release()
        
    # Destroy all the vehicles and sensors
    for actor in world.get_actors().filter('*vehicle*'):
        actor.destroy()
    for actor in world.get_actors().filter('*pedestrian*'):
        actor.destroy()
    for actor in world.get_actors().filter('*sensor*'):
        actor.destroy()

    for proc in psutil.process_iter():
        if proc.name() == "CarlaUE4.exe" or proc.name() == "CarlaUE4-Win64-Shipping.exe": proc.kill()