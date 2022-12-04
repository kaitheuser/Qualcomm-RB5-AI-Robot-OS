import matplotlib.pyplot as plt
import numpy as np
import csv
from math import ceil

height = width = 3
cell_size = 0.05
clearance = 0.3
coverage  = 0.3
CLEARANCE = 1
PATH = 2
COVERAGE = 3

clearance_grid = int(clearance / cell_size)
map_height = int(height / cell_size)
mao_width  = int(width / cell_size)
map = np.zeros((map_height, mao_width))
map[: clearance_grid, :] = CLEARANCE
map[map_height - clearance_grid : map_height, :] = CLEARANCE
map[:, : clearance_grid] = CLEARANCE
map[:, map_height - clearance_grid : map_height] = CLEARANCE

def ground_position_transform(ground_pos):
    '''Transform from ground position to map position.'''
    x, y = ground_pos
    return (height - y, x)

def array_position_transform(map_pos):
    '''Transform from map position to array position/index.'''
    x, y = map_pos
    row, col = int(x / cell_size), int(y / cell_size)
    return (row + 2, col)

def ground_to_array_transform(ground_pos):
    '''Transform from ground position to array index.'''
    return array_position_transform(
        ground_position_transform(
            ground_pos
        )
    )    

def generate_coverage(curr_pos):
    x, y = curr_pos
    coverage_grid = ceil(coverage / cell_size)
    for i in range(-coverage_grid, coverage_grid + 1):
        for j in range(-coverage_grid, coverage_grid + 1):
            cx, cy = x + i, y + j
            if map[cx, cy] == 0:
                map[cx, cy] = COVERAGE

pos = []
# Load telemetry csv file  
with open("/Users/kaitheuser/Downloads/Qualcomm-RB5-AI-Robot-roomba_OS/telemetry_data/20221127-1858_ambiguity_best.csv", 'r') as f:
    csvreader = csv.reader(f)
    for id, row in enumerate(csvreader):
        # Store timestamp, vehicle pose, landmark positions, tagID
        if id % 2 == 0:
            pos.append(ground_to_array_transform((float(row[1]), float(row[2]))))

for x, y in pos:
    map[x, y] = PATH
    generate_coverage((x, y))

plt.imshow(map)
plt.title('Visualization of Robot Coverage Map')
plt.savefig('roomba_coverage_005.png')
plt.show()

area = np.count_nonzero(map != CLEARANCE)
uncovered_area = np.count_nonzero(map == 0)
print (f'Coverage percentage is {round((area - uncovered_area) / area * 100, 4)}%.')

