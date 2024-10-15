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
    Process(target=CARLA, args=()).start() # Open Carla in a subprocess
    
    for t in range(10): print('>', end=''); time.sleep(1) # Wait 10 seconds for Carla to launched
    print(' Started.')

    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    spawn_points = world.get_map().get_spawn_points()
    bp_lib = world.get_blueprint_library()
    print('>>>>')

    



    ''' other road users'''
    my_vehicles = []
    for i in range(150): 
        vehicle_bp = random.choice(bp_lib.filter('vehicle')) 
        new_vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))    
        world.wait_for_tick()
        if new_vehicle is not None:
            new_vehicle.set_autopilot(True)
            my_vehicles.append(new_vehicle)
    ''''''

    # my_pedestrians = []
    # for i in range(150): 
    #     pedestrian_bp = random.choice(bp_lib.filter('walker.pedestrian.*'))
    #     spawn_point = random.choice(spawn_points)
    #     transform = carla.Transform(spawn_point.location, spawn_point.rotation)
    #     pedestrian_actor = world.try_spawn_actor(pedestrian_bp, transform)    
    #     world.wait_for_tick()
    #     if pedestrian_actor is not None:
    #         my_pedestrians.append(pedestrian_actor)


    number_of_walkers = 200
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









    # Get list of map layers
    map_layers = [
        carla.MapLayer.Buildings,
        carla.MapLayer.Foliage,
        carla.MapLayer.Decals,
        carla.MapLayer.ParkedVehicles,
        carla.MapLayer.Props,
        carla.MapLayer.Walls,
        carla.MapLayer.Particles,
        carla.MapLayer.StreetLights
    ]
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


 # Define the spectator
    spectator = world.get_spectator()

    while True:
        current_time = time.time()
        time_difference_layer = current_time - last_layer_change_time
        time_difference_weather = current_time - last_weather_change_time

        if time_difference_layer >= 3:
            # Randomly select map layers to load and unload
            layer_to_load = random.choice(map_layers)
            layer_to_unload = random.choice(map_layers)
            
            # Unload selected layer
            world.unload_map_layer(layer_to_unload)
            
            # Load selected layer
            world.load_map_layer(layer_to_load)
            world.unload_map_layer(carla.MapLayer.Props)

            last_layer_change_time = current_time
        

        if time_difference_weather >= 3:
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

        world.wait_for_tick()    


    for proc in psutil.process_iter():
        if proc.name() == "CarlaUE4.exe" or proc.name() == "CarlaUE4-Win64-Shipping.exe": proc.kill()
