import glob
import bagpy
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from bagpy import bagreader

# Window filter that smooths the data and returns both the upper and lower bound for that window
def window_filter(x, window_size=5):
    upper_bound = np.array(pd.Series(x).rolling(window_size).max())[window_size-1:]
    lower_bound = np.array(pd.Series(x).rolling(window_size).min())[window_size-1:]
    mean = np.array(pd.Series(x).rolling(window_size).mean())[window_size-1:]
    return upper_bound, lower_bound, mean


# Get all the files
bag_files = glob.glob("Hover/*.bag")

# For each bag file
for i, file_name in enumerate(bag_files):

    # Load the bag
    print("Loading: {}".format(file_name))
    bag = bagreader(file_name)

    # get the list of topics
    print("Available topics: ")
    print(bag.topic_table)

    print("Processing")
    # Read specific topics and convert to panda
    state_vector_msgs   = bag.message_by_topic("/current_state")
    rpyt_msgs           = bag.message_by_topic("/rpyt_command")
    state_vector_data   = pd.read_csv(state_vector_msgs)
    rpyt_data           = pd.read_csv(rpyt_msgs)

    # Get the fields we are interested in from the current state
    pos_time        = np.array(state_vector_data["Time"])
    x_data          = np.array(state_vector_data["state_vector_0"])
    z_data          = np.array(state_vector_data["state_vector_2"])

    # Get the fiels we are interested in from the rpyt data
    command_time    = np.array(rpyt_data["Time"])
    control_mode    = np.array(rpyt_data["ctrl_mode"])
    pitch_data      = np.array(rpyt_data["pitch"])
    thurst_data     = np.array(rpyt_data["thrust"]) 

    # Convert from down to up
    z_data = z_data * -1

    # Convert from Epoch time to seconds
    pos_time = pos_time - pos_time[0]
    command_time = command_time - command_time[0]

    # Find the times we are interested in (seconds)
    start_time = 60
    end_time = 480
    end_time = end_time + start_time

    p_start     = np.argmax(pos_time > start_time)
    p_end       = np.argmax(pos_time > end_time)
    c_start     = np.argmax(command_time > start_time)
    c_end       = np.argmax(command_time > end_time)

    # Crop all data to start and end at right time
    pos_time        = pos_time[p_start:p_end]
    x_data          = x_data[p_start:p_end]
    z_data          = z_data[p_start:p_end]

    command_time    = command_time[c_start:c_end]
    control_mode    = control_mode[c_start:c_end]
    pitch_data      = pitch_data[c_start:c_end]
    thurst_data     = thurst_data[c_start:c_end]

    # Reset time to start at 0
    pos_time = pos_time - pos_time[0]
    command_time = command_time - command_time[0]

    # Smooth the data
    win_size_p = 1000
    win_size_c = 100
    x_upper, x_lower, x_mean = window_filter(x_data, window_size=win_size_p)
    z_upper, z_lower, z_mean = window_filter(z_data, window_size=win_size_p)
    p_upper, p_lower, p_mean = window_filter(pitch_data, window_size=win_size_c)
    t_upper, t_lower, t_mean = window_filter(thurst_data, window_size=win_size_c)

    # Crop the time to fit inside the filter
    pos_time = pos_time[win_size_p-1:]
    command_time = command_time[win_size_c-1:]

    # Generate a labl
    label_text = ""
    if "JOZI_10m_hover" in file_name:
        label_text = "Base drone"
    elif "JOZI_10m_weight" in file_name:
        label_text = "Artificial Weight"
    elif "SUIT" in file_name:
        label_text = "Haptic Suit"

    # Plot the data
    print("Plotting")

    plt.figure(3)
    plt.plot(pos_time, x_mean, c="C{}".format(i), label=label_text)
    plt.fill_between(pos_time, x_lower, x_upper, color="C{}".format(i), alpha=0.3)
    plt.title("X Position")
    plt.ylabel("Position (m)")
    plt.xlabel("Time (s)")
    plt.legend()

    plt.figure(4)
    plt.plot(pos_time, z_mean, c="C{}".format(i), label=label_text)
    plt.fill_between(pos_time, z_lower, z_upper, color="C{}".format(i), alpha=0.3)
    plt.title("Z Position")
    plt.ylabel("Position (m)")
    plt.xlabel("Time (s)")
    plt.legend()

    plt.figure(6)
    plt.plot(command_time, p_mean, c="C{}".format(i), label=label_text)
    plt.fill_between(command_time, p_lower, p_upper, color="C{}".format(i), alpha=0.3)
    plt.title("Pitch Commands")
    plt.ylabel("Pitch (?)")
    plt.xlabel("Time (s)")
    plt.legend()

    plt.figure(7)
    plt.plot(command_time, t_mean, c="C{}".format(i), label=label_text)
    plt.fill_between(command_time, t_lower, t_upper, color="C{}".format(i), alpha=0.3)
    plt.title("Thrust Commands")
    plt.ylabel("Thrust (N)")
    plt.xlabel("Time (s)")
    plt.legend()

    print("----------------")
plt.show()
