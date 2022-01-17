from mpl_toolkits import mplot3d
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("/home/lbarnett/ros2_ws/src/rcraicer/rcraicer_mppi_test/training_data/gazebo_data_train.csv")

fig = plt.figure()
ax = fig.add_subplot(2, 2, 1, projection='3d')

ax.scatter(df["steer"], df["throttle"], df["accel_x"], c=df["accel_x"], cmap='viridis')
ax.set_xlabel("steer")
ax.set_ylabel("throttle")
ax.set_zlabel("accel_x")


ax = fig.add_subplot(2, 2, 3, projection='3d')

ax.scatter(df["steer"], df["throttle"], df["accel_y"], c=df["accel_y"], cmap='viridis')
ax.set_xlabel("steer")
ax.set_ylabel("throttle")
ax.set_zlabel("accel_y")


ax = fig.add_subplot(2, 2, 2, projection='3d')

ax.scatter(df["steer"], df["throttle"], df["act_roll"], c=df["act_roll"], cmap='viridis')
ax.set_xlabel("steer")
ax.set_ylabel("throttle")
ax.set_zlabel("act_roll")

ax = fig.add_subplot(2, 2, 4, projection='3d')

ax.scatter(df["steer"], df["throttle"], df["accel_yaw"], c=df["accel_yaw"], cmap='viridis')
ax.set_xlabel("steer")
ax.set_ylabel("throttle")
ax.set_zlabel("accel_yaw")


plt.show()       
plt.close()


