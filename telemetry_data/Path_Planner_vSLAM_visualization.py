# Import necessary library
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
import csv
import numpy as np
from itertools import count

'''
Initialize Lists
'''
data = []                               # Data list that stores timestamp, vehicle pose, landmark positions, tagID
covs = []                               # Covariance list that stores covariance
wp_x = []                               # Waypoint x-coordinate list
wp_y = []                               # Waypoint y-coordinate list

pos_x = []                              # Robot position x-coordinate
pos_y = []                              # Robot position y-coordinate
planned_path_x = []                     # A* planned path waypoint x-coordinate
planned_path_y = []                     # A* planned path waypoint y-coordinate
tag_x = []                              # Detected April-tag position x-coordinate
tag_y = []                              # Detected April-tag position y-coordinate
observed = []                           # Observed April-tag ID
 
'''
Load telemetry csv file
[[t, x, y, theta, mx1, my1, mx2, my2, ..., tagID1,tagID2, ...],
 [covariance at time t]]
'''
# Load telemetry csv file  
with open("/home/rosws/src/rb5_ros/telemetry_data/20221121-1025_astar_path.csv", 'r') as f:
# with open("/home/rosws/src/rb5_ros/telemetry_data/20221121-1036_voronoi_path.csv", 'r') as f:
    csvreader = csv.reader(f)
    for id, row in enumerate(csvreader):
        # Store timestamp, vehicle pose, landmark positions, tagID
        if id % 2 == 0:
            # Convert timestamp, and vehicle pose strings to float
            if len(row) <= 4:
                for idx in range(0, len(row)):
                    row[idx] = float(row[idx])
            else:
                for idx in range(0, len(row)):
                    num_Landmarks = (len(row) - 4) // 3
                    # Convert landmark positions strings to float
                    if idx <= (3 + 2*num_Landmarks):
                        row[idx] = float(row[idx])
                    # Convert tagID strings to int
                    else:
                        row[idx] = int(row[idx])
            data.append(row)
        # Store covariance
        else:
            # Convert covariance strings to float
            for idx in range(0, len(row)):
                row[idx] = float(row[idx]) 
            covs.append(row)

'''
Load planned path csv file
[[x, y, theta]]
'''
# Load planned path csv
with open("/home/rosws/src/rb5_ros/waypoints/a_star_waypoints.csv", 'r') as f:
# with open("/home/rosws/src/rb5_ros/waypoints/voronoi_waypoints.csv", 'r') as f:
    csvreader = csv.reader(f)
    for row in csvreader:
        # Get x-y coordinate
        x, y, _ = row
        # Append to waypoint lists
        wp_x.append(round(float(x),2))
        wp_y.append(round(float(y),2))
        

'''
Defined map landmarks, wall, and obstacles
'''
# Define truth landmark positions
tagX = [0.0, 0.61, 2.44, 3.05, 3.05, 2.44, 0.61, 0.0, 1.305, 1.53, 1.755, 1.53]
tagY = [0.61, 0.0, 0.0, 0.61, 2.44, 3.05, 3.05, 2.44, 1.505, 1.34, 1.505, 1.67]
tag_mat = np.array([tagX,tagY])

# Define wall
wall_x = [0.0, 3.05, 3.05, 0.0, 0.0]
wall_y = [0.0, 0.0, 3.05, 3.05, 0.0]

# Define obstacle
obs_x = [1.305, 1.755, 1.755, 1.305, 1.305]
obs_y = [1.34, 1.34, 1.67, 1.67, 1.34]


'''
Initialize animated plot
'''
fig, ax = plt.subplots()
ax.plot(pos_x, pos_y)
fig.set_size_inches(10, 8)
counter = count(0,1)


def update(i):
    '''
    Define animated plot function
    '''
    
    # Import global vairables
    global tag_x
    global tag_y
    global wp_x
    global wp_y
    global observed
    
    # Get counter index
    idx = next(counter)
    
    # Plot data and covariance until the end of line
    if idx < len(data):
        
        # Extract timestamp, vehicle pose, landmark positions, tagID at time t
        data_t = data[idx]
        # Extract covariance at time t
        cov_t = covs[idx]
        # Determine Covariance square matrix size
        cov_size = int(np.sqrt(len(cov_t)))
        # Turns the flatten covariance matrix into a 2D-matrix
        cov_t = np.array(cov_t).reshape(cov_size, cov_size)
        plt.cla()
        
        # Update vehicle position data at time t
        if len(data_t) < 5:
            pos_x.append(data_t[1])
            pos_y.append(data_t[2])
        # Update landmark information at time t
        else:
            # Determine number of landmarks
            num_Landmarks = (len(data_t) - 4) // 3
            pos_x.append(data_t[1])
            pos_y.append(data_t[2])
            
            # Calculate the eigenvector and eigenvalues of the covariance to plot the covariance ellipse
            cov_v = cov_t[:2, :2]
            eigenvalues_v, eigenvectors_v = np.linalg.eig(cov_v)
            ellipse = patches.Ellipse((data_t[1], data_t[2]), width= eigenvalues_v[1], height=eigenvalues_v[0], angle=np.rad2deg(np.arccos(eigenvectors_v[0, 0])), alpha = 0.5)
            ax.add_patch(ellipse)
            
            # Extract landmark tagIDs
            landmark_ids = data_t[(4 + 2*num_Landmarks): (4 + 3*num_Landmarks)]
            
            # Update detected landmark tagIDs
            for landmark_id in landmark_ids:
                
                # If not observed before
                if landmark_id not in observed:
    
                    # Add new landmark info
                    observed.append(landmark_id)
                    j = observed.index(landmark_id)
                    tag_x.append(data_t[4 + 2*j])
                    tag_y.append(data_t[5 + 2*j])
                    
                    # Calculate the eigenvector and eigenvalues of the covariance to plot the covariance ellipse
                    cov_l = cov_t[3+2*j:5+2*j, 3+2*j:5+2*j]
                    eigenvalues_l, eigenvectors_l = np.linalg.eig(cov_l)
                    ellipse = patches.Ellipse((data_t[4 + 2*j], data_t[5 + 2*j]), width= eigenvalues_l[1], height=eigenvalues_l[0], angle=np.rad2deg(np.arccos(eigenvectors_l[0, 0])), alpha = 0.5, color='pink')
                    ax.add_patch(ellipse)
 
                else:
                    
                    # Update landmark info
                    j = observed.index(landmark_id)
                    tag_x[j] = data_t[4 + 2*j]
                    tag_y[j] = data_t[5 + 2*j]
                    
                    # Calculate the eigenvector and eigenvalues of the covariance to plot the covariance ellipse
                    cov_l = cov_t[3+2*j:5+2*j, 3+2*j:5+2*j]
                    eigenvalues_l, eigenvectors_l = np.linalg.eig(cov_l)
                    ellipse = patches.Ellipse((data_t[4 + 2*j], data_t[5 + 2*j]), width= eigenvalues_l[1], height=eigenvalues_l[0], angle=np.rad2deg(np.arccos(eigenvectors_l[0, 0])), alpha = 0.5, color='pink')
                    ax.add_patch(ellipse)
            
            # Plot detected tags
            ax.scatter(tag_x, tag_y, c ="blue", linewidths = 2, marker ="s", edgecolor ="purple", s = 50, label = "Detected April Tag")
        
        # Update planned path waypoints
        if idx < len(wp_x):
            planned_path_x.append(wp_x[idx])
            planned_path_y.append(wp_y[idx])
        
        # Plot robot positions and planned path
        ax.plot(pos_x, pos_y , '--g*', label = "Robot Position")
        ax.plot(planned_path_x, planned_path_y, '-r^', label = "Planned Path")
        
        # Plot start and end points
        ax.scatter(wp_x[0], wp_y[0], c ="cyan", linewidths = 2, marker ="D", edgecolor ="blue", s = 200, label = "Start")
        ax.scatter(wp_x[-1], wp_y[-1], c ="yellow", linewidths = 2, marker ="*", edgecolor ="red", s = 200, label = "Goal")
        
        # Plot April-tag position ground truth
        ax.scatter(tagX, tagY, linewidths = 2, marker ="x", s = 50, label = "True April Tag")
        # Annotate the True April-tags
        for i, tagIDx in enumerate([0,1,2,3,4,5,6,7,2,4,6,3]):
            ax.annotate(tagIDx, (tagX[i], tagY[i]), fontsize=16)
        
        # Plot and annotate April-tag measured position
        for i, tagID in enumerate(observed):
            if tagID == 8:
                tagID = 2
            if tagID == 9:
                tagID = 4
            if tagID == 10:
                tagID = 6
            if tagID == 11:
                tagID = 3
            ax.annotate(tagID, (tag_x[i], tag_y[i]), fontsize=16)
            
        # Plot walls and obstacles
        ax.plot(wall_x, wall_y, ':k', label = "Wall")
        ax.plot(obs_x, obs_y, '-.b', label = "Obstacles")
        
        # Label plot x and y axis and include title
        ax.set_xlabel('Width, x [m]')
        ax.set_ylabel('Length, y [m]')
        ax.set_title('A* Path Planner with Visual SLAM')
        # ax.set_title('Voronoi Path Planner with Visual SLAM')
        ax.legend(bbox_to_anchor=(0.80, 1.16), loc='upper left')
        
        
# Update plot every 200 ms.
ani = FuncAnimation(fig=fig, func=update, interval = 200)
plt.show()
