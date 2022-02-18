import glob
import bagpy
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from bagpy import bagreader

# Window filter that smooths the data and returns both the upper and lower bound for that window
def window_filter(x, window_size=5):

    # Create the rolling window to have all the same data as the first point
    window = np.full(window_size, x[0])

    # Create the output data
    mean_data   = np.zeros(len(x))
    upper       = np.zeros(len(x))
    lower       = np.zeros(len(x))

    # Run through the array
    for i, d in enumerate(x):
        mean_data[i]    = np.average(window)
        upper[i]        = mean_data[i] + np.std(window)
        lower[i]        = mean_data[i] - np.std(window)

        # Roll the window
        window = np.roll(window, 1)
        window[0] = d
        
    return mean_data, upper, lower

parser = argparse.ArgumentParser()
parser.add_argument('--test_duration', type=int, default=600,                help='Test duration (s)')
parser.add_argument('--wait_time',     type=int, default=10,                 help='Time (s) after computer control mode before you start the test')
parser.add_argument('--folder',        type=str, default='Hover/third_run/', help='Folder where bag files are stored')
parser.add_argument('--window_size',   type=int, default=100,                help='The size of the window filter')
args = parser.parse_args()

# Get all the files
bag_files = glob.glob("{}*.bag".format(args.folder))

# For each bag file
for bag_file_index, file_name in enumerate(bag_files):

    # Init for each loop
    figure_count = 0

    # Load the bag

    bag = bagreader(file_name)

    # get the list of topics
    print("Available topics: ")
    print(bag.topic_table)

    print("Working on: {}".format(file_name))
    print("\tLoading data")

    # Read specific topics and convert to panda
    state_vector_msgs   = bag.message_by_topic("/current_state")
    rpyt_msgs           = bag.message_by_topic("/rpyt_command")
    reference_msgs      = bag.message_by_topic("/reference_state")
    mavros_state_msgs   = bag.message_by_topic("/mavros/state")
    state_vector_data   = pd.read_csv(state_vector_msgs)
    rpyt_data           = pd.read_csv(rpyt_msgs)
    reference_data      = pd.read_csv(reference_msgs)
    mavros_state_data   = pd.read_csv(mavros_state_msgs)

    print("\tProcessing data")

    # Get the position (3), velocity(3), and attiude (3)
    state_vector_length = len(state_vector_data["state_vector_0"])
    vehicle_data        = {}
    vehicle_labels      = ["x_pos", "y_pos", "z_pos", "x_vel", "y_vel", "z_vel", "roll", "pitch", "yaw", "dt"]
    vehicle_indices     = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12]
    for label, index in zip(vehicle_labels, vehicle_indices):
        vehicle_data[label] = np.array(state_vector_data["state_vector_{}".format(index)])
    vehicle_data["Time"] = np.array(state_vector_data["Time"])

    # Convert the z position and velocity to be up
    vehicle_data["z_pos"] = vehicle_data["z_pos"] * -1
    vehicle_data["z_vel"] = vehicle_data["z_vel"] * -1

    # Get the command data (4)
    command_data        = {}
    command_labels      = ["roll", "pitch", "yaw", "thrust"]
    for label in command_labels:
        command_data[label] = np.array(rpyt_data["{}".format(label)])
    command_data["Time"] = np.array(rpyt_data["Time"])

    # Get the setpoints 
    setpoint_data        = {}
    setpoint_labels      = ["x_pos", "y_pos", "z_pos", "x_vel", "y_vel", "z_vel"]
    setpoint_names       = ["pn", "pe", "pd", "vn", "ve", "vd"]
    for label, name in zip(setpoint_labels, setpoint_names):
        setpoint_data[label] = np.array(reference_data["{}".format(name)])
    setpoint_data["Time"] = np.array(reference_data["Time"])

    # Convert the z position and velocity to be up
    setpoint_data["z_pos"] = setpoint_data["z_pos"] * -1
    setpoint_data["z_vel"] = setpoint_data["z_vel"] * -1

    # Get the computer control mode data
    control_mode_data    = {}
    control_mode_labels  = ["armed", "mode"]
    for label in control_mode_labels:
        control_mode_data[label] = np.array(mavros_state_data["{}".format(label)])
    control_mode_data["Time"] = np.array(mavros_state_data["Time"])

    print("\tCropping only test data")

    # Determine when the drone was first set to computer control mode
    cc_time = 0
    for i in range(len(control_mode_data["mode"])):
        a = control_mode_data["armed"][i]
        m = control_mode_data["mode"][i] 
          
        if (a) and ("CMODE(25)" in m):
            # Define when computer control was actually started
            cc_time = control_mode_data["Time"][i]
            break 
    
    # Get the indices where to crop data from
    start_time = cc_time + args.wait_time
    end_time = cc_time + args.test_duration + args.wait_time
    vehicle_crop_s_index = np.argmax(vehicle_data["Time"]>=start_time)
    command_crop_s_index = np.argmax(command_data["Time"]>=start_time)
    setpoint_crop_s_index = np.argmax(setpoint_data["Time"]>=start_time)
    control_crop_s_index = np.argmax(control_mode_data["Time"]>=start_time)

    vehicle_crop_e_index = np.argmax(vehicle_data["Time"]>=end_time)
    command_crop_e_index = np.argmax(command_data["Time"]>=end_time)
    setpoint_crop_e_index = np.argmax(setpoint_data["Time"]>=end_time)
    control_crop_e_index = np.argmax(control_mode_data["Time"]>=end_time)

    # Crop the data
    for key in vehicle_data:
        vehicle_data[key] = vehicle_data[key][vehicle_crop_s_index:vehicle_crop_e_index]
    for key in command_data:
        command_data[key] = command_data[key][command_crop_s_index:command_crop_e_index]
    for key in setpoint_data:
        setpoint_data[key] = setpoint_data[key][setpoint_crop_s_index:setpoint_crop_e_index]
    for key in control_mode_data:
        control_mode_data[key] = control_mode_data[key][control_crop_s_index:control_crop_e_index]
       
    # Convert to seconds
    print("\tConverting from Unix time to seconds")
    vehicle_data["Time"] = vehicle_data["Time"] - vehicle_data["Time"][0]
    command_data["Time"] = command_data["Time"] - command_data["Time"][0]
    setpoint_data["Time"] = setpoint_data["Time"] - setpoint_data["Time"][0]
    control_mode_data["Time"] = control_mode_data["Time"] - control_mode_data["Time"][0]

    print("\tPlotting data")   
    
    # Plot the position data
    time = vehicle_data["Time"]

    # X data
    plt.figure(figure_count)
    average_x, upper_x, lower_x = window_filter(vehicle_data["x_pos"], window_size=args.window_size)
    plt.plot(time, average_x, c="C{}".format(bag_file_index), label=file_name)
    plt.fill_between(time, lower_x, upper_x, color="C{}".format(bag_file_index), alpha=0.3)
    figure_count += 1

    # Y data
    plt.figure(figure_count)
    average_y, upper_y, lower_y = window_filter(vehicle_data["y_pos"], window_size=args.window_size)
    plt.plot(time, average_x, c="C{}".format(bag_file_index), label=file_name)
    plt.fill_between(time, lower_y, upper_y, color="C{}".format(bag_file_index), alpha=0.3)
    figure_count += 1

    # Z data
    plt.figure(figure_count)
    average_z, upper_z, lower_z = window_filter(vehicle_data["z_pos"], window_size=args.window_size)
    plt.plot(time, average_z, c="C{}".format(bag_file_index), label=file_name)
    plt.fill_between(time, lower_z, upper_z, color="C{}".format(bag_file_index), alpha=0.3)
    figure_count += 1


figure_count = 0

# Plot the position setpoints data
time = setpoint_data["Time"]

# X data
plt.figure(figure_count)
data_x = setpoint_data["x_pos"]
plt.plot(time, data_x, c="Black", label="Setpoint")
plt.title("X Position")
plt.ylabel("Position (m)")
plt.xlabel("Time (s)")
plt.ylim([-0.5, 0.5])
plt.legend()
figure_count += 1

# Y data
plt.figure(figure_count)
data_y = setpoint_data["y_pos"]
plt.plot(time, data_y, c="Black", label="Setpoint")
plt.title("Y Position")
plt.ylabel("Position (m)")
plt.xlabel("Time (s)")
plt.ylim([-0.5, 0.5])
plt.legend()
figure_count += 1

# Z data
plt.figure(figure_count)
data_z = setpoint_data["z_pos"]
plt.plot(time, data_z, c="Black", label="Setpoint")
plt.title("Z Position")
plt.ylabel("Position (m)")
plt.xlabel("Time (s)")
plt.ylim([0.25, 1.25])
plt.legend()
figure_count += 1
plt.show()



# # TODO Add vicon data to the plot