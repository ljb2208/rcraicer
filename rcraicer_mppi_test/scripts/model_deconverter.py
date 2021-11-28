import numpy as np
import torch
import torch.nn as nn
from dyn_model import DynModel

input_path = "/home/lbarnett/ros2_ws/src/rcraicer/rcraicer_mppi_test/models/dynmodel_states4.pth_good"
conv_path = "/home/lbarnett/catkin_ws/src/autorally/autorally_control/src/path_integral/params/models/gazebo_nnet_09_12_2018.npz"
output_path = "/home/lbarnett/ros2_ws/src/rcraicer/rcraicer_mppi_test/models/autorally_states.pth"

# arr = np.load(input_path, allow_pickle=True, encoding='bytes')

# for f in arr.files:
#     print(f)
#     print(arr[f])

conv = np.load(conv_path)

model = DynModel()
model.load_state_dict(torch.load(input_path))
sd = model.state_dict()

sd["lin1.weight"] = torch.tensor(conv["dynamics_W1"].astype(np.float32))
sd["lin2.weight"] = torch.tensor(conv["dynamics_W2"].astype(np.float32))
sd["lin_last.weight"] = torch.tensor(conv["dynamics_W3"].astype(np.float32))
sd["lin1.bias"] = torch.tensor(conv["dynamics_b1"].astype(np.float32))
sd["lin2.bias"] = torch.tensor(conv["dynamics_b2"].astype(np.float32))
sd["lin_last.bias"] = torch.tensor(conv["dynamics_b3"].astype(np.float32))


# # Save data to numpy array, each channel is saved individually as an array in row major order.
# model_dict = {"dynamics_W1": sd["lin1.weight"].numpy().astype(np.float64), 
#                 "dynamics_W2": sd["lin2.weight"].numpy().astype(np.float64), 
#                 "dynamics_W3": sd["lin_last.weight"].numpy().astype(np.float64),                 
#                 "dynamics_b1": sd["lin1.bias"].numpy().astype(np.float64),
#                 "dynamics_b2": sd["lin2.bias"].numpy().astype(np.float64),
#                 "dynamics_b3": sd["lin_last.bias"].numpy().astype(np.float64)}

# np.savez(output_path, **model_dict)

torch.save(sd, output_path)


# data = np.load(input_path)

# for f in data.files:
#     print(f)
#     print("Shape: " + str((data[f].shape)))
#     # print(data[f])

print("here")