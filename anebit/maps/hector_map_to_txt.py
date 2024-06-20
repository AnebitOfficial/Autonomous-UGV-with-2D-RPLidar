import yaml
import matplotlib.image as mpimg
import numpy as np

def load_map_metadata(yaml_file):
    with open(yaml_file, 'r') as file:
        map_metadata = yaml.safe_load(file)
    return map_metadata


def load_occupancy_data(pgm_file):
    image = mpimg.imread(pgm_file,format="pgm")
    return image


def extract_occupied_cells(occupancy_data, threshold=200):
    # Occupied cells are those where the value is above the threshold
    return np.argwhere(occupancy_data == threshold)

def grid_to_world(indices, resolution, origin):
    # Convert grid indices to world coordinates
    x_origin, y_origin = origin[0], origin[1]
    world_coordinates = indices * resolution + np.array([x_origin, y_origin])
    return world_coordinates

def convert_map_to_point_cloud(yaml_file, pgm_file):
    metadata = load_map_metadata(yaml_file)
    occupancy_data = load_occupancy_data(pgm_file)
    occupied_indices_obstacle = extract_occupied_cells(occupancy_data,0)
    occupied_indices_free = extract_occupied_cells(occupancy_data,254)
    point_cloud_obstacle = grid_to_world(occupied_indices_obstacle, metadata['resolution'], metadata['origin'])
    point_cloud_free = grid_to_world(occupied_indices_free, metadata['resolution'], metadata['origin'])
    return point_cloud_obstacle, point_cloud_free
    
    
yaml_file = 'maps/hector_map.yaml'
pgm_file = 'maps/hector_map.pgm'
point_cloud_obstacle, point_cloud_free = convert_map_to_point_cloud(yaml_file, pgm_file)

s = ""
for i in range(len(point_cloud_obstacle)):
    s += '(' + str(1000*point_cloud_obstacle[i][0]) + ',' + str(1000*point_cloud_obstacle[i][1]) + "),"
s = s[:-1]
with open("maps/hector_map.txt","w") as file:
    file.write(s)
