import numpy as np

from math import pi
from tqdm import tqdm
from equations import foward_kinematics

# User defined
Fx = 10
Fy = 10
Fz = 0

# Define the resolution
resolution = 3
acceptance_threshold = 0.05

# Constants
kw = 2

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

                                check1 = abs(result_Fx - Fx) < acceptance_threshold
                                check2 = abs(result_Fy - Fy) < acceptance_threshold
                                check3 = abs(result_Fz - Fz) < acceptance_threshold


                                if (check1 and check2 and check3):
                                    print("Theta1: {}\nTheta2: {}\nTheta3: {}\nTheta4: {}\nOmega1: {}\nOmega2: {}\nOmega3: {}\nOmega4: {}".format(theta_1, theta_2, theta_3, theta_4, omega_1, omega_2, omega_3, omega_4))
                                    print("------------------")
