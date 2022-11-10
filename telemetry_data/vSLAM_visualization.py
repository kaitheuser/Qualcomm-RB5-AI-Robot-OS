# Import necessary library
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import csv
import numpy as np
from itertools import count

# Load telemetry csv file
data = []
covs = []
with open("/home/rosws/src/rb5_ros/telemetry_data/20221108-1255sq_best_path.csv", 'r') as f:
# with open("/home/rosws/src/rb5_ros/telemetry_data/20221109-1316_msqbestpath.csv", 'r') as f:
# with open("/home/rosws/src/rb5_ros/telemetry_data/20221108-1353oct_best_path.csv", 'r') as f:
    csvreader = csv.reader(f)
    for id, row in enumerate(csvreader):
        if id % 2 == 0:
            if len(row) <= 4:
                for idx in range(0, len(row)):
                    row[idx] = float(row[idx])
            else:
                for idx in range(0, len(row)):
                    num_Landmarks = (len(row) - 4) // 3
                    if idx <= (3 + 2*num_Landmarks):
                        row[idx] = float(row[idx])
                    else:
                        row[idx] = int(row[idx])
            data.append(row)
        else:
            for idx in range(0, len(row)):
                row[idx] = float(row[idx]) 
            covs.append(row)
             
# Initialize robot positions as list
pos_x = []
pos_y = []
tag_x = []
tag_y = []
observed = []

# Define landmarks and wayppoints
## Square
tagX = [-0.61, 0.46, 1.84, 2.44, 2.44, 1.53, 0.32, -0.61]
tagY = [0.21, -0.61, -0.61, 0.19, 1.51, 2.44, 2.44, 1.75]
wp_x = [0.0, 1.0, 1.0, 0.0]
wp_y = [0.0, 0.0, 1.0, 1.0]
## Multiple Round Square
# tagX = 0.305 - np.array([-1.22, -0.15, 1.23, 1.83, 1.83, 0.92, -0.29, -1.22])
# tagX = tagX.tolist()
# tagY = np.array([0.21, -0.61, -0.61, 0.19, 1.51, 2.44, 2.44, 1.75]) -0.305
# tagY = tagY.tolist()
# wp_x = [0.0, 1.0, 1.0, 0.0]
# wp_y = [0.0, 0.0, 1.0, 1.0]
## Octagon
# tagX = [-1.22, -0.15, 1.23, 1.83, 1.83, 0.92, -0.29, -1.22]
# tagY = [0.21, -0.61, -0.61, 0.19, 1.51, 2.44, 2.44, 1.75]
# wp_x = [0.0, 0.61, 1.22, 1.22, 0.61, 0.0, -0.61, -0.61]
# wp_y = [0.0, 0.0, 0.61, 1.22, 1.83, 1.83, 1.22, 0.61]


fig, ax = plt.subplots()
ax.plot(pos_x, pos_y)

counter = count(0,1)
def update(i):
    global tag_x
    global tag_y
    global wp_x
    global wp_y
    global observed
    
    idx = next(counter)*2
    if idx < len(data):
        data_t = data[idx]
        plt.cla()
        if len(data_t) < 5:
            pos_x.append(data_t[1])
            pos_y.append(data_t[2])
        else:
            num_Landmarks = (len(data_t) - 4) // 3
            pos_x.append(data_t[1])
            pos_y.append(data_t[2])
            
            landmark_ids = data_t[(4 + 2*num_Landmarks): (4 + 3*num_Landmarks)]
            for landmark_id in landmark_ids:
                if landmark_id not in observed:
                    observed.append(landmark_id)
                    j = observed.index(landmark_id)
                    tag_x.append(data_t[4 + 2*j])
                    tag_y.append(data_t[5 + 2*j])
                else:
                    j = observed.index(landmark_id)
                    tag_x[j] = data_t[4 + 2*j]
                    tag_y[j] = data_t[5 + 2*j]
            
            ax.scatter(tag_x, tag_y, c ="blue", linewidths = 2, marker ="s", edgecolor ="purple", s = 100, label = "April Tag")
        
        ax.plot(pos_x, pos_y , '--g*', label = "Robot Position")
        ax.scatter(wp_x, wp_y, c ="yellow", linewidths = 2, marker ="^", edgecolor ="red", s = 200, label = "Waypoint")
        
        ax.scatter(tagX, tagY, linewidths = 2, marker ="x", s = 100, label = "True April Tag")
        
        ax.set_xlabel('Width, x [m]')
        ax.set_ylabel('Length, y [m]')
        ax.set_title('Visual SLAM')
        ax.legend(bbox_to_anchor=(0.80, 1.16), loc='upper left')
        
# Update plot every 200 ms.
ani = FuncAnimation(fig=fig, func=update, interval = 200)
plt.show()