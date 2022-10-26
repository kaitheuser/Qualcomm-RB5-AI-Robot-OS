# Import necessary library
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from itertools import count
import pandas as pd

# Load telemetry csv file
df = pd.read_csv("/home/rosws/src/rb5_ros/telemetry_data/filtered_video_path.csv")
# Initialize robot positions as list
pos_x = []
pos_y = []
wp_x = [0, 0.5, 0.5]
wp_y = [0, 0, 1.0,]
tag_x = [1.0, -0.3]
tag_y = [0, 1.0]

# Initialize plot
fig, ax = plt.subplots()
ax.plot(pos_x, pos_y)

counter = count(0,1)
def update(i):
    idx = next(counter)
    pos_x.append(df.iloc[idx, 1])
    pos_y.append(df.iloc[idx, 2])
    plt.cla()
    ax.plot(pos_x, pos_y , '--g*', label = "Robot Position")
    ax.scatter(wp_x, wp_y, c ="yellow", linewidths = 2, marker ="^", edgecolor ="red", s = 200, label = "Waypoint")
    ax.scatter(tag_x, tag_y, c ="blue", linewidths = 2, marker ="s", edgecolor ="purple", s = 100, label = "April Tag")
    ax.set_xlim(-0.4, 1.2)
    ax.set_ylim(-0.4, 1.2)
    ax.set_xlabel('Length, x [m]')
    ax.set_ylabel('Length, y [m]')
    ax.set_title('Scaled Down Map - 50%')
    ax.legend()
# Update plot every 200 ms.
ani = FuncAnimation(fig=fig, func=update, interval = 200)
plt.show()