import numpy as np
import pandas as pd
from pydrake.all import LinearQuadraticRegulator, DiscreteTimeLinearQuadraticRegulator

from pydrake.all import (
    DiagramBuilder,
    LinearSystem,
    FiniteHorizonLinearQuadraticRegulator,
    FiniteHorizonLinearQuadraticRegulatorOptions,
    Simulator,
)


# Given parameters

# --Physical Parameters--
# iris
m = 1.535
l = 0.25
I_xxt = 0.029138900000000002
I_yyt = 0.030227416
I_zzt = 0.056327416000000005
I_zzp = 0.000273104
# x500
# m = 2.0
# l = 0.25
# I_xxt = 0.02166666666666667
# I_yyt = 0.02166666666666667
# I_zzt = 0.04000000000000001
# I_zzp = 1.1928e-4

# --Aerodynamic Parameters--
# iris
k_tau = 0.06
gamma = 0.007811651
# x500
# k_tau = 0.016
# gamma = 4.16e-4 #x500

# --Equilibrium Values--
# x500 new
# p_eq = 0
# q_eq = 6.2937
# r_eq = 20.4653
# w1_eq = 591.6514
# w2_eq = 733.6477
# w3_eq = 591.6514
# nx_eq = 0.0
# ny_eq = 0.2939
# nz_eq = 0.9558

# x500 old
# p_eq = 0
# q_eq = 2.5350
# r_eq = 10
# w1_eq = 738.0
# w2_eq = 522.0
# w3_eq = 738.0
# w4_eq = 0.0
# nx_eq = 0.0
# ny_eq = 0.2855
# nz_eq = 0.9584

# iris
p_eq = 0
q_eq = 2.25361451700798
r_eq = 11.694423666728527
w1_eq = 600.4122857965222
w2_eq = 768.5277258195483
w3_eq = 600.4122857965222
w4_eq = 0.0
nx_eq = 0.0
ny_eq = 0.1892268837006408
nz_eq = 0.9819333920816344

# Calculate a_const,b_const,c_const,d_const
a_const = ((I_xxt - I_zzt) * r_eq / I_xxt) + I_zzp * (
    w1_eq + w2_eq + w3_eq + w4_eq
) / I_xxt
b_const = (I_xxt - I_zzt) * q_eq / I_xxt
c_const = (I_zzt - I_xxt) * p_eq / I_xxt
d_const = -gamma / I_zzt

# Define matrices A, B, Q, R
# Define the matrix A without deltat
A = np.array(
    [
        [1, a_const, b_const, 0, 0],
        [-a_const, 1, c_const, 0, 0],
        [0, 0, d_const + 1, 0, 0],
        [0, -nz_eq, ny_eq, 1, r_eq],
        [nz_eq, 0, -nx_eq, -r_eq, 1],
    ]
)
# A = np.array(
#     [
#         [       1,  a_const,      b_const,     0,    0, 0, 0],
#         [-a_const,        1,      c_const,     0,    0, 0, 0],
#         [       0,        0,  d_const + 1,     0,    0, 0, 0],
#         [       0,    -nz_eq,       ny_eq,     1, r_eq, 0, 0],
#         [   nz_eq,        0,       -nx_eq, -r_eq,    1, 0, 0],
#         [       0,        0,            0,     0,    0, 1, 1],
#         [       0,        0,            0,     0,    0, 0, 1],
#     ]
# )

B = np.array(
    [
        [0, l / I_xxt, 0],
        [-l / I_xxt, 0, l / I_xxt],
        [k_tau / I_zzt, -k_tau / I_zzt, k_tau / I_zzt],
        [0, 0, 0],
        [0, 0, 0],
    ]
)
# B = np.array(
#     [
#         [            0,      l / I_xxt,             0],
#         [   -l / I_xxt,              0,     l / I_xxt],
#         [k_tau / I_zzt, -k_tau / I_zzt, k_tau / I_zzt],
#         [            0,              0,             0],
#         [            0,              0,             0],
#         [            0,              0,             0],
#         [      nz_eq/m,        nz_eq/m,       nz_eq/m],
#     ]
# )
# B = np.array(
#     [
#         [            0,      l / I_xxt,             0],
#         [   -l / I_xxt,              0,     l / I_xxt],
#         [k_tau / I_zzt, -k_tau / I_zzt, k_tau / I_zzt],
#         [            0,              0,             0],
#         [            0,              0,             0],
#     ]
# )
# B = np.array(
#     [
#         [        0,     l / I_xxt,              0],
#         [l / I_xxt,             0,              0],
#         [        0, -k_tau / I_zzt, k_tau / I_zzt],
#         [        0,              0,             0],
#         [        0,              0,             0],
#     ]
# )

# builder = DiagramBuilder()
# system = builder.AddSystem(LinearSystem(A, B, np.zeros((2,)), np.zeros((1,)), time_period=1.0))
# context = system.CreateDefaultContext()

# Ensure Q is symmetric
# Q = np.array(
#     [
#         [40, 0, 0,   0,   0,  0,  0],
#         [0, 40, 0,   0,   0,  0,  0],
#         [0, 0, 45,   0,   0,  0,  0],
#         [0, 0, 0,   130,   0,  0,  0],
#         [0, 0, 0,   0,  200,  0,  0],
#         [0, 0, 0,   0,   0,  0.000001,  0],
#         [0, 0, 0,   0,   0,  0,  0.000001],
#     ]
# )

Q = np.array(
    [
        [300, 0, 0, 300, 0],
        [0, 100, 0, 0, 0],
        [0, 0, 500, 0, 0],
        [300, 0, 0, 900, 0],
        [0, 0, 0, 0, 500],
    ]
)


# R = np.array([[20, 0, 0],
#               [0, 250, 0],
#               [0, 0, 20]])
R = np.array(
    [
        [500, 0, 0],
        [0, 500, 0],
        [0, 0, 500],
    ]
)
# Calculate the LQR gain
result = LinearQuadraticRegulator(A, B, Q, R)


# Print the result[0] in C++ style initialization format
result_cpp_format = (
    "K = {\n"
    + ",\n".join(
        ["  {" + ", ".join(f"{x:.8e}" for x in row) + "}" for row in result[0]]
    )
    + "\n};"
)

print(result_cpp_format)


def compute_control_input(x):
    """
    Compute the control input vector u given the state vector x.

    :param x: State vector
    :return: Control input vector u
    """
    # Calculate the control input u = -Kx
    y = [float(x[0]), float(x[1]) - 2.53, float(x[2]), float(x[3]) - 0.2855]
    u = -K @ y
    return u


# Read the data from data.csv
# data = pd.read_csv('/home/kuldeep/Desktop/tutorial_ROS2/data.csv', header=0)

# Extract the relevant columns for the state vector (assuming columns are in order: Roll_Rate, Pitch_Rate, nx, ny)
# state_vectors = data.iloc[:, [0, 1, 2, 3]].values

# Compute control inputs for each state vector
# control_inputs = np.array([compute_control_input(x) for x in state_vectors])

# Create a DataFrame for the control inputs
# control_inputs_df = pd.DataFrame(control_inputs, columns=['u1', 'u2'])

# Save the control inputs to data_python.csv
# control_inputs_df.to_csv('/home/kuldeep/Desktop/tutorial_ROS2/data_python.csv', index=False)

# Created/Modified files during execution:
# print('/home/kuldeep/Desktop/tutorial_ROS2/data_python.csv')
