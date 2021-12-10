using Test

include("../../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

# Module for testing of simulation
include("SimulationTesting.jl")
using .SimulationTesting


# inertia matrix
inertia = [
    45000 -1000 300
    -1000 1200 750
    300 750 50000
]

# Earth-Centered frame (constant value)
ECI_frame = Frame(
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
)


# Sampling period of simulation (second)
Tsampling = 1.0
# Time length of simulation (second)
simulation_time = 5400 * 2

# Numbers of simulation data
datanum = round(Int, simulation_time/Tsampling) + 1;

# define a orbit info
orbitinfo = Orbit.OrbitInfo(Orbit.OrbitalElements(0, 0, 6370e+3 + 400e3, 1, 0, 0), ECI_frame)

# Dynamics model (mutable struct)
model = RigidBody.RigidBodyModel(inertia)

# Initialize data array
initvalue = TimeLine.InitData(
    [0, 0, 0, 1],
    [0, 0, 0],
    ECI_frame
)

distconfig = DisturbanceConfig(gravitygradient = true)

println("Begin simulation!")
@time (time, simdata, orbitdata) = runsimulation(model, ECI_frame, initvalue, orbitinfo, distconfig, simulation_time, Tsampling)
println("Simulation is completed!")

@test SimulationTesting.quaternion_constraint(simdata.quaternion)

fig1 = PlotRecipe.angularvelocities(time, simdata.angularvelocity)
display(fig1)

fig2 = PlotRecipe.quaternions(time, simdata.quaternion)
display(fig2)

fig3 = PlotRecipe.frame_gif(time, Tsampling, ECI_frame, simdata.bodyframe, Tgif = 100, FPS = 8)
display(fig3)
