import numpy as np
from PIL import Image


# map_path = "/home/lbarnett/catkin_ws/src/autorally/autorally_control/src/path_integral/params/maps/gazebo/gazebo_map.npz"
# map_path = "/home/lbarnett/catkin_ws/src/autorally/autorally_control/src/path_integral/params/maps/ccrf/ccrf_track.npz"
# map_path = "/home/lbarnett/ros2_ws/src/rcraicer/costmap.npz"
# map_path = "/home/lbarnett/ros2_ws/src/rcraicer/costmap_new.npz"
# map_path = "/home/lbarnett/catkin_ws/src/autorally/autorally_control/src/path_integral/params/maps/marietta_costmap_07_25_2019.npz"
map_path = "/home/lbarnett/ros2_ws/src/rcraicer/costmap.npz"
output_path = "/home/lbarnett/ros2_ws/src/rcraicer/costmap_test.png"
# output_path = "/home/lbarnett/catkin_ws/src/autorally/autorally_control/src/path_integral/params/maps/marietta_costmap_07_25_2019.png"
# output_path = "/home/lbarnett/ros2_ws/src/rcraicer/costmap_new5.npz"

text_path = "/home/lbarnett/catkin_ws/src/autorally/autorally_control/src/path_integral/params/maps/gazebo/gazebo_map.txt"

data = np.load(map_path)

# mult = np.full(data["channel0"].shape, 0.01)
# zeros = np.zeros(data["channel0"].shape, dtype=np.float32)
# newChannel = np.multiply(data["channel0"], mult)

channel0 = data["channel0"]


for i in range(0, len(channel0)):
    if channel0[i] > 1.0:
        channel0[i] = 1.0


xBounds = data["xBounds"]
yBounds = data["yBounds"]
ppm = data["pixelsPerMeter"]

w = round((float(yBounds[1]) - float(yBounds[0]))*float(ppm[0]))
h = round((float(xBounds[1]) - float(xBounds[0]))*float(ppm[0]))

img = channel0.reshape((w,h))
img = img[::-1,:]

img = np.array(img*255.0, dtype=np.uint8)
img = Image.fromarray(img)
img.save(output_path)

# Save data to numpy array, each channel is saved individually as an array in row major order.
# track_dict = {"xBounds": np.array([-311.3,  173.3], np.float32), 
#                 "yBounds": np.array([-112.3,  196.9], np.float32),
#                 "pixelsPerMeter":np.array([10.0], np.float32), 
#                 "channel0":newChannel,
#                 "channel1":data["channel1"].astype(np.float32),
#                 "channel2":data["channel2"].astype(np.float32),
#                 "channel3":data["channel3"].astype(np.float32)}

# np.savez(output_path, **track_dict)


# output = open(text_path, "w")


print("Channel len")
print(len(data["channel0"]))

print("channel0")
print(np.amin(data["channel0"]))
print(np.amax(data["channel0"]))
print(np.average(data["channel0"]))

print("channel1")
print(np.amin(data["channel1"]))
print(np.amax(data["channel1"]))
print(np.average(data["channel1"]))

print("channel2")
print(np.amin(data["channel2"]))
print(np.amax(data["channel2"]))
print(np.average(data["channel2"]))

print("channel3")
print(np.amin(data["channel3"]))
print(np.amax(data["channel3"]))
print(np.average(data["channel3"]))

channel_len = len(data["channel0"])

# print("X_in")
# print(str(data["X_in"]))

# print("X_out")
# print(str(data["X_out"]))

# print("X_cen")
# print(str(data["X_cen"]))

# print("X_cen_smooth")
# print(str(data["X_cen_smooth"]))

# print("Y_in")
# print(str(data["Y_in"]))

# print("Y_out")
# print(str(data["Y_out"]))

# print("Y_cen")
# print(str(data["Y_cen"]))

# print("Y_cen_smooth")
# print(str(data["Y_cen_smooth"]))

# print("W_cen")
# print(str(data["W_cen"]))

# print("W_cen_smooth")
# print(str(data["W_cen_smooth"]))

print("pixelsPerMeter")
print(str(data["pixelsPerMeter"]))

print("xBounds")
print(str(data["xBounds"]))

print("yBounds")
print(str(data["yBounds"]))

print("filterChannel")
print(str(data["filterChannel"]))




# mins = [100000., 100000., 100000., 100000.]
# maxs = [-100000., -100000., -100000., -100000.]
# channels = ["channel0", "channel1", "channel2", "channel3"]

# for i in range(0, channel_len):
#     for y in (range(0, 3)):
#         val = data[channels[y]][i]

#         if val > maxs[y]:
#             maxs[y] = val
        
#         if val < mins[y]:
#             mins[y] = val


# print(mins)
# print(maxs)

# output.write("0:1:2:3\n")

# for i in range(0,channel_len - 1):
#     output.write(str(data["channel0"][i]))   
#     output.write(":")
#     output.write(str(data["channel1"][i]))   
#     output.write(":")
#     output.write(str(data["channel2"][i]))   
#     output.write(":")
#     output.write(str(data["channel3"][i]))    
#     output.write(":\n")





# output.close()

print("here")