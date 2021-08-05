import numpy as np
from PIL import Image


# input_path = "/home/lbarnett/catkin_ws/src/autorally/autorally_control/src/path_integral/params/models/gazebo_nnet_09_12_2018.npz"
input_path = "/home/lbarnett/catkin_ws/src/autorally/autorally_control/src/path_integral/params/models/autorally_nnet_09_12_2018.npz"
# input_path = "/home/lbarnett/ros2_ws/src/rcraicer/rcraicer_mppi_test/models/dynmodel_states4.npz"
# input_path = "/home/lbarnett/ros2_ws/src/rcraicer/rcraicer_mppi_test/models/dynmodel1.npz"

data = np.load(input_path)

for f in data.files:
    print(f)
    print("Shape: " + str((data[f].shape)))
    # print(data[f])

print("here")