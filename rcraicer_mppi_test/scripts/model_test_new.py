import numpy as np
import torch
import torch.nn as nn
from dyn_model import DynModel

from mpl_toolkits import mplot3d
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

model1_path = "/home/lbarnett/ros2_ws/src/rcraicer/rcraicer_mppi_test/models/gazebo_dynmodel_states1.pth"
model2_path = "/home/lbarnett/ros2_ws/src/rcraicer/rcraicer_mppi_test/models/autorally_states.pth"

model1 = DynModel()
model1.load_state_dict(torch.load(model1_path))
model1.eval()

model2 = DynModel()
model2.load_state_dict(torch.load(model2_path))
model2.eval()

data = [0., 4., 1., 0., 0., 0.]

results = np.empty((20000, 6), dtype=np.float32)
results2 = np.empty((20000, 6), dtype=np.float32)

resultsIndex = 0


for x in range (-100, 100):
    for y in range (0, 100):
        data[4] = x/100.
        data[5] = y/100.

        x_data = torch.tensor(data).to(dtype=torch.float32)            
        output = model1.forward(x_data)

        output2 =model2.forward((x_data))

        results[resultsIndex][0] = data[4]
        results[resultsIndex][1] = data[5]
        results[resultsIndex][2] = output[0]
        results[resultsIndex][3] = output[1]
        results[resultsIndex][4] = output[2]
        results[resultsIndex][5] = output[3]

        results2[resultsIndex][0] = data[4]
        results2[resultsIndex][1] = data[5]
        results2[resultsIndex][2] = output2[0]
        results2[resultsIndex][3] = output2[1]
        results2[resultsIndex][4] = output2[2]
        results2[resultsIndex][5] = output2[3]

        resultsIndex += 1

        # print (output)

df = pd.DataFrame(data=results, dtype=results.dtype, columns=["steer", "throttle", "roll_out", "u_x_out", "u_y_out", "yaw_mder_out"])   
df2 = pd.DataFrame(data=results2, dtype=results.dtype, columns=["steer", "throttle", "roll_out", "u_x_out", "u_y_out", "yaw_mder_out"])   

# df.to_csv("test.csv")

fig = plt.figure()
ax = fig.add_subplot(4, 2, 1, projection='3d')

ax.scatter(df["steer"], df["throttle"], df["u_x_out"], c=df["u_x_out"], cmap='viridis')
ax.set_xlabel("steer")
ax.set_ylabel("throttle")
ax.set_zlabel("u_x")


ax = fig.add_subplot(4, 2, 3, projection='3d')

ax.scatter(df["steer"], df["throttle"], df["u_y_out"], c=df["u_y_out"], cmap='viridis')
ax.set_xlabel("steer")
ax.set_ylabel("throttle")
ax.set_zlabel("u_y")


ax = fig.add_subplot(4, 2, 5, projection='3d')

ax.scatter(df["steer"], df["throttle"], df["roll_out"], c=df["roll_out"], cmap='viridis')
ax.set_xlabel("steer")
ax.set_ylabel("throttle")
ax.set_zlabel("roll")

ax = fig.add_subplot(4, 2, 7, projection='3d')

ax.scatter(df["steer"], df["throttle"], df["yaw_mder_out"], c=df["yaw_mder_out"], cmap='viridis')
ax.set_xlabel("steer")
ax.set_ylabel("throttle")
ax.set_zlabel("yaw_mder")

# plot results2
ax = fig.add_subplot(4, 2, 2, projection='3d')

ax.scatter(df2["steer"], df2["throttle"], df2["u_x_out"], c=df2["u_x_out"], cmap='viridis')
ax.set_xlabel("steer")
ax.set_ylabel("throttle")
ax.set_zlabel("u_x")


ax = fig.add_subplot(4, 2, 4, projection='3d')

ax.scatter(df2["steer"], df2["throttle"], df2["u_y_out"], c=df2["u_y_out"], cmap='viridis')
ax.set_xlabel("steer")
ax.set_ylabel("throttle")
ax.set_zlabel("u_y")


ax = fig.add_subplot(4, 2, 6, projection='3d')

ax.scatter(df["steer"], df2["throttle"], df2["roll_out"], c=df2["roll_out"], cmap='viridis')
ax.set_xlabel("steer")
ax.set_ylabel("throttle")
ax.set_zlabel("roll")

ax = fig.add_subplot(4, 2, 8, projection='3d')

ax.scatter(df["steer"], df2["throttle"], df2["yaw_mder_out"], c=df2["yaw_mder_out"], cmap='viridis')
ax.set_xlabel("steer")
ax.set_ylabel("throttle")
ax.set_zlabel("yaw_mder")



plt.show()       
plt.close()


