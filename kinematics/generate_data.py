
import os
import csv
import random
import numpy as np

from math import pi
from tqdm import tqdm
from equations import foward_kinematics

# Define the 
resolution = 4

print("This will generate {} datapoints".format(pow(resolution,8)))

# Constants
kw = 2

# Create data dir
if not os.path.exists("./data"):
    os.makedirs("./data")  

# Create two CSV files
input_training_file = open('./data/training_input.csv', 'w')
output_training_file = open('./data/training_output.csv', 'w')
writer_training_in = csv.writer(input_training_file)
writer_training_out = csv.writer(output_training_file)

for theta_1 in tqdm(np.linspace(0, 2*pi, resolution), position=1, leave=False):
    for theta_2 in tqdm(np.linspace(0, 2*pi, resolution), position=2, leave=False):
        for theta_3 in tqdm(np.linspace(0, 2*pi, resolution), position=3, leave=False):
            for theta_4 in tqdm(np.linspace(0, 2*pi, resolution), position=4, leave=False):
                for omega_1 in tqdm(np.linspace(0, 10, resolution), position=5, leave=False):
                    for omega_2 in tqdm(np.linspace(0, 10, resolution), position=6, leave=False):
                        for omega_3 in tqdm(np.linspace(0, 10, resolution), position=7, leave=False):
                            for omega_4 in tqdm(np.linspace(0, 10, resolution), position=8, leave=False):

                                result_Fx, result_Fy, result_Fz = foward_kinematics(kw,
                                                                                    theta_1,
                                                                                    theta_2,
                                                                                    theta_3,
                                                                                    theta_4,
                                                                                    omega_1,
                                                                                    omega_2,
                                                                                    omega_3,
                                                                                    omega_4)

                                writer_training_in.writerow([theta_1, theta_2, theta_3, theta_4, omega_1, omega_2, omega_3, omega_4])
                                writer_training_out.writerow([result_Fx, result_Fy, result_Fz])

input_training_file.close()
output_training_file.close()

input_testing_file = open('./data/testing_input.csv', 'w')
output_testing_file = open('./data/testing_output.csv', 'w')
writer_testing_in = csv.writer(input_testing_file)
writer_testing_out = csv.writer(output_testing_file)

# Generate a training dataset
for j in range(1000):

    # Generate random theta and omegas
    thetas = []
    omegas = []
    for i in range(4):
        theta = random.uniform(0, 2*pi)
        thetas.append(theta)
        omega = random.uniform(0, 10)
        omegas.append(omega)

    # Compute the results
    result_Fx, result_Fy, result_Fz = foward_kinematics(kw,
                                                        thetas[0],
                                                        thetas[1],
                                                        thetas[2],
                                                        thetas[3],
                                                        omegas[0],
                                                        omegas[1],
                                                        omegas[2],
                                                        omegas[3])

    # Save to file
    writer_testing_in.writerow(thetas + omegas)
    writer_testing_out.writerow([result_Fx, result_Fy, result_Fz])

input_testing_file.close()
output_testing_file.close()

print("Done!")