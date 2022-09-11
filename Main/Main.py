import sys
sys.path.insert(0, 'H:\Trajectory_Simulator\Main')
import RunTest as Simulation

inputPath = "H:\Trajectory_Simulator\Main\Inputs.xls"
coefficientPath = "H:\Trajectory_Simulator\External Simulation\SU2\Coefficients.xlsx"


start = Simulation.TrajectorySimulation(1000, inputPath, coefficientPath)
mySimulation = start.RunSimulation()