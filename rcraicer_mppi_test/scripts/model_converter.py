import numpy as np
import torch
import torch.nn as nn
from dyn_model import DynModel


input_path = "/home/lbarnett/ros2_ws/src/rcraicer/rcraicer_mppi_test/models/dynmodel_gazebo_states1.pth"
output_path = "/home/lbarnett/ros2_ws/src/rcraicer/rcraicer_mppi_test/models/dynmodel_gazebo_states1.npz"

model = DynModel()
model.load_state_dict(torch.load(input_path))
sd = model.state_dict()

l1w = sd["lin1.weight"].numpy()


# Save data to numpy array, each channel is saved individually as an array in row major order.
model_dict = {"dynamics_W1": sd["lin1.weight"].numpy().astype(np.float64), 
                "dynamics_W2": sd["lin2.weight"].numpy().astype(np.float64), 
                "dynamics_W3": sd["lin_last.weight"].numpy().astype(np.float64),                 
                "dynamics_b1": sd["lin1.bias"].numpy().astype(np.float64),
                "dynamics_b2": sd["lin2.bias"].numpy().astype(np.float64),
                "dynamics_b3": sd["lin_last.bias"].numpy().astype(np.float64)}

np.savez(output_path, **model_dict)



print(sd)

# data = np.load(input_path)

# for f in data.files:
#     print(f)
#     print("Shape: " + str((data[f].shape)))
#     # print(data[f])

print("here")