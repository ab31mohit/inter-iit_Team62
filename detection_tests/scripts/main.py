import numpy as np
import pandas as pd
from pydrake.all import LinearQuadraticRegulator,DiscreteTimeLinearQuadraticRegulator

from pydrake.all import (DiagramBuilder, LinearSystem, FiniteHorizonLinearQuadraticRegulator,
                       FiniteHorizonLinearQuadraticRegulatorOptions, Simulator)


# Given parameters
I_xxt = 0.02166666666666667
I_yyt = 0.02166666666666667
I_zzt = 0.04000000000000001
I_zzp = 1.1928e-4
l = 0.25
r_eq = 8.5
w1_eq = 738.0
w2_eq = 522.0
w3_eq = 738.0
w4_eq = 0.0

nz_eq = 0.9583

# Calculate a_const
a_const = ((I_xxt - I_zzt) * r_eq / I_xxt) + I_zzp * (w1_eq + w2_eq + w3_eq + w4_eq) / I_xxt

# Define matrices A, B, Q, R
# Define the matrix A without deltat
A = np.array([
    [1,        a_const,  0,        0        ],
    [-a_const, 1,        0,        0        ],
    [0,        -nz_eq,   1,        r_eq     ],
    [nz_eq,    0,        -r_eq,    1        ]
])

B = np.array([
    [0, l / I_xxt],
    [l / I_xxt, 0],
    [0, 0],
    [0, 0]
])

# builder = DiagramBuilder()
# system = builder.AddSystem(LinearSystem(A, B, np.zeros((2,)), np.zeros((1,)), time_period=1.0))
# context = system.CreateDefaultContext()

# Ensure Q is symmetric
Q = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 20, 0],
    [0, 0, 0, 20]
])

R = np.array([
    [1, 0],
    [0, 1]
])

# Calculate the LQR gain
result = LinearQuadraticRegulator(A, B, Q, R)


# Print the result[0] in C++ style initialization format
result_cpp_format = "K = {\n" + ",\n".join(
    ["  {" + ", ".join(f"{x:.8e}" for x in row) + "}" for row in result[0]]
) + "\n};"

print(result_cpp_format)

def compute_control_input(x):
    """
    Compute the control input vector u given the state vector x.
    
    :param x: State vector
    :return: Control input vector u
    """
    # Calculate the control input u = -Kx
    y = [float(x[0]), float(x[1])-2.53, float(x[2]), float(x[3])-0.2855]
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