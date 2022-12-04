# Import necessary library
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
import csv
import numpy as np
from itertools import count

# Load telemetry csv file
data = []
covs = []
    
# with open("/home/rosws/src/rb5_ros/telemetry_data/20221121-0524_selected_fatest_path.csv", 'r') as f:
with open("/home/rosws/src/rb5_ros/telemetry_data/20221108-1255sq_best_path.csv", 'r') as f:
# with open("/home/rosws/src/rb5_ros/telemetry_data/20221110-1459_msqbestpath.csv", 'r') as f:
# with open("/home/rosws/src/rb5_ros/telemetry_data/20221108-1358oct_best_path.csv", 'r') as f:
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
tag_mat = np.array([tagX,tagY])

## Multiple Round Square
# tagX = np.array([-1.22, -0.15, 1.23, 1.83, 1.83, 0.92, -0.29, -1.22]) + 0.305
# tagX = tagX.tolist()
# tagY = np.array([0.21, -0.61, -0.61, 0.19, 1.51, 2.44, 2.44, 1.75]) - 0.305
# tagY = tagY.tolist()
# tag_mat = np.array([tagX,tagY])
# wp_x = [0.0, 1.0, 1.0, 0.0]
# wp_y = [0.0, 0.0, 1.0, 1.0]

## Octagon
# tagX = [-1.22, -0.15, 1.23, 1.83, 1.83, 0.92, -0.29, -1.22]
# tagY = [0.21, -0.61, -0.61, 0.19, 1.51, 2.44, 2.44, 1.75]
# tag_mat = np.array([tagX,tagY])
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
    
    idx = next(counter)
    if idx < len(data):
        data_t = data[idx]
        cov_t = covs[idx]
        cov_size = int(np.sqrt(len(cov_t)))
        cov_t = np.array(cov_t).reshape(cov_size, cov_size)
        plt.cla()
        if len(data_t) < 5:
            pos_x.append(data_t[1])
            pos_y.append(data_t[2])
                
        else:
            num_Landmarks = (len(data_t) - 4) // 3
            pos_x.append(data_t[1])
            pos_y.append(data_t[2])
            
            cov_v = cov_t[:2, :2]
            eigenvalues_v, eigenvectors_v = np.linalg.eig(cov_v)
            ellipse = patches.Ellipse((data_t[1], data_t[2]), width= eigenvalues_v[1], height=eigenvalues_v[0], angle=np.rad2deg(np.arccos(eigenvectors_v[0, 0])), alpha = 0.5)
            ax.add_patch(ellipse)
            
            
            landmark_ids = data_t[(4 + 2*num_Landmarks): (4 + 3*num_Landmarks)]
            for landmark_id in landmark_ids:
                if landmark_id not in observed:
                    observed.append(landmark_id)
                    j = observed.index(landmark_id)
                    tag_x.append(data_t[4 + 2*j])
                    tag_y.append(data_t[5 + 2*j])
                    
                    cov_l = cov_t[3+2*j:5+2*j, 3+2*j:5+2*j]
                    eigenvalues_l, eigenvectors_l = np.linalg.eig(cov_l)
                    ellipse = patches.Ellipse((data_t[4 + 2*j], data_t[5 + 2*j]), width= eigenvalues_l[1], height=eigenvalues_l[0], angle=np.rad2deg(np.arccos(eigenvectors_l[0, 0])), alpha = 0.5, color='pink')
                    ax.add_patch(ellipse)
 
                else:
                    j = observed.index(landmark_id)
                    tag_x[j] = data_t[4 + 2*j]
                    tag_y[j] = data_t[5 + 2*j]
                    
                    cov_l = cov_t[3+2*j:5+2*j, 3+2*j:5+2*j]
                    eigenvalues_l, eigenvectors_l = np.linalg.eig(cov_l)
                    ellipse = patches.Ellipse((data_t[4 + 2*j], data_t[5 + 2*j]), width= eigenvalues_l[1], height=eigenvalues_l[0], angle=np.rad2deg(np.arccos(eigenvectors_l[0, 0])), alpha = 0.5, color='pink')
                    ax.add_patch(ellipse)
            
            ax.scatter(tag_x, tag_y, c ="blue", linewidths = 2, marker ="s", edgecolor ="purple", s = 50, label = "April Tag")
        
        ax.plot(pos_x, pos_y , '--g*', label = "Robot Position")
        ax.scatter(wp_x, wp_y, c ="yellow", linewidths = 2, marker ="^", edgecolor ="red", s = 200, label = "Waypoint")
        
        ax.scatter(tagX, tagY, linewidths = 2, marker ="x", s = 50, label = "True April Tag")
        for i, tagID in enumerate(observed):
            ax.annotate(tagID, (tag_x[i], tag_y[i]), fontsize=16)
            
        for i, tagIDx in enumerate([0,1,2,3,4,5,7,8]):
            ax.annotate(tagIDx, (tagX[i], tagY[i]), fontsize=16)
        
        ax.set_xlabel('Width, x [m]')
        ax.set_ylabel('Length, y [m]')
        ax.set_title('Visual SLAM')
        ax.legend(bbox_to_anchor=(0.80, 1.16), loc='upper left')
        
    elif idx == len(data):
        # Calculate error
        data_t = data[idx-1]
        errors = np.zeros((len(observed)))
        for tagID in observed:
            j = observed.index(tagID)
            landmark_j_pos_SLAM = np.array([data_t[4 + 2*j], data_t[5 + 2*j]])
            j_true = [0,1,2,3,4,5,7,8].index(tagID)
            landmark_j_pos_true = tag_mat[:,j_true]
            error = np.linalg.norm(landmark_j_pos_SLAM-landmark_j_pos_true)
            errors[j_true] = error
        # Display errors and the average error
        for tagID in [0,1,2,3,4,5,7,8]:
            j_true = [0,1,2,3,4,5,7,8].index(tagID)
            print(f'Error of Landmark {tagID}: {errors[j_true]}')
        print(f'Average Error of Landmarks: {np.mean(errors)} m')
        pass
        
# Update plot every 200 ms.
ani = FuncAnimation(fig=fig, func=update, interval = 200)
plt.show()
