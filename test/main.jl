using Test

include("../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

# Sampling period of simulation (second)
Tsampling = 1e-2
# Time length of simulation (second)
simulation_time = 1000

# Initialize the simulation configurations
(simconfig, ECI_frame) = initsimulation(simulation_time, Tsampling)

# Set the dynamics model
model = setdynamicsmodel("./test/spacecraft.yml",)

# define a orbit info
orbitinfo = initorbitinfo("./test/orbit2.yml", ECI_frame)

# Set disturbance torque
distconfig = DisturbanceConfig(gravitygradient = true)

# Initialize data array
initvalue = TimeLine.InitData(
    [0, 0, 0, 1],
    [0, 0, 0],
    ECI_frame
)

# run simulation
println("Begin simulation!")
@time (time, simdata, orbitdata) = runsimulation(model, initvalue, orbitinfo, distconfig, simconfig)
println("Completed!")

@test Evaluation.quaternion_constraint(simdata.quaternion)

fig1 = PlotRecipe.angularvelocities(time, simdata.angularvelocity)
# fig1 = PlotRecipe.angularvelocities(time, simdata.angularvelocity, timerange = (0, 10))
display(fig1)

fig2 = PlotRecipe.quaternions(time, simdata.quaternion)
display(fig2)

# Plot of the body frame with respect to ECI frame
fig3 = PlotRecipe.framegif(time, ECI_frame, simdata.rollpitchyawframe, Tgif = 20, FPS = 8)
display(fig3)

# Plot of the animation of LVLH frame
fig4 = PlotRecipe.framegif(time, ECI_frame, orbitdata.LVLH,  Tgif = 20, FPS = 8)
display(fig4)
