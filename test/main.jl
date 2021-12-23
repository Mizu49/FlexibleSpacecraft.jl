using Test

include("../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

# Sampling period of simulation (second)
Tsampling = 1e-2
# Time length of simulation (second)
simulation_time = 5400

# Set the dynamics model
model = setdynamicsmodel("./test/spacecraft.yml",)

# Earth-Centered frame (constant value)
ECI_frame = Frame(
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1]
)

# Initialize data array
initvalue = TimeLine.InitData(
    [0, 0, 0, 1],
    [0, 0, 0],
    ECI_frame
)

# define a orbit info
orbitinfo = initorbitinfo("./test/orbit.yml", ECI_frame)

distconfig = DisturbanceConfig(gravitygradient = true)

println("Begin simulation!")
# run simulation
@time (time, simdata, orbitdata) = runsimulation(model, ECI_frame, initvalue, orbitinfo, distconfig, simulation_time, Tsampling)

println("Completed!")

@test Evaluation.quaternion_constraint(simdata.quaternion)

fig1 = PlotRecipe.angularvelocities(time, simdata.angularvelocity)
display(fig1)

fig2 = PlotRecipe.quaternions(time, simdata.quaternion)
display(fig2)

# Plot of the body frame with respect to ECI frame
fig3 = PlotRecipe.frame_gif(time, Tsampling, ECI_frame, simdata.bodyframe, Tgif = 30, FPS = 8)
display(fig3)

# Plot of the animation of LVLH frame
# fig4 = PlotRecipe.frame_gif(time, Tsampling, ECI_frame, orbitdata.LVLH, Tgif = 60, FPS = 8)
# display(fig4)
