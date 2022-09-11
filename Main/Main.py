import sys
sys.path.insert(0, 'H:\Trajectory_Simulator\Main')

import RunTest as Simulation

# Paths to the input values and the drag and lift coefficients 
inputPath = "H:\Trajectory_Simulator\Main\Inputs.xls"
coefficientPath = "H:\Trajectory_Simulator\External Simulation\SU2\Coefficients.xlsx"

# Runs the simulation
start = Simulation.TrajectorySimulation(1000, inputPath, coefficientPath)
mySimulation = start.RunSimulation()