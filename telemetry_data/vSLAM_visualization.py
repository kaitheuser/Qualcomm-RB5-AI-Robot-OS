# Import necessary library
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import csv
from itertools import count

# Load telemetry csv file
data = []
with open("/home/rosws/src/rb5_ros/telemetry_data/20221108-101425_path.csv", 'r') as f:
    csvreader = csv.reader(f)
    for row in csvreader:
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
             
# Initialize robot positions as list
pos_x = []
pos_y = []
tag_x = []
tag_y = []

# Initialize plot
fig, ax = plt.subplots()
ax.plot(pos_x, pos_y)

counter = count(0,1)
def update(i):
    idx = next(counter)
    num_Landmarks = (len(data[idx]) - 4) // 3
    pos_x.append(data[idx, 1])
    pos_y.append(data[idx, 2])
    tag_x.append(data[idx, 4:(3 + 2*num_Landmarks + 1):2])
    tag_x.append(data[idx, 5:(3 + 2*num_Landmarks + 1):2])
    plt.cla()
    ax.plot(pos_x, pos_y , '--g*', label = "Robot Position")
    # ax.scatter(wp_x, wp_y, c ="yellow", linewidths = 2, marker ="^", edgecolor ="red", s = 200, label = "Waypoint")
    ax.scatter(tag_x, tag_y, c ="blue", linewidths = 2, marker ="s", edgecolor ="purple", s = 100, label = "April Tag")
    ax.set_xlim(-1.0, 4.0)
    ax.set_ylim(-1.0, 4.0)
    ax.set_xlabel('Width, x [m]')
    ax.set_ylabel('Length, y [m]')
    ax.set_title('Visual SLAM')
    ax.legend()
# Update plot every 200 ms.
ani = FuncAnimation(fig=fig, func=update, interval = 200)
plt.show()