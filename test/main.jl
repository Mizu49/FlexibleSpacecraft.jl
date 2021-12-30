using Test

include("../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

# Sampling period of simulation (second)
Tsampling = 1e-2
# Time length of simulation (second)
simulation_time = 1000

# Set the dynamics model
model = setdynamicsmodel("./test/spacecraft.yml",)

(simconfig, ECI_frame) = initsimulation(simulation_time, Tsampling)

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
@time (time, simdata, orbitdata) = runsimulation(model, ECI_frame, initvalue, orbitinfo, distconfig, simconfig)

println("Completed!")

@test Evaluation.quaternion_constraint(simdata.quaternion)

fig1 = PlotRecipe.angularvelocities(time, simdata.angularvelocity)
# fig1 = PlotRecipe.angularvelocities(time, simdata.angularvelocity, timerange = (0, 10))
display(fig1)

fig2 = PlotRecipe.quaternions(time, simdata.quaternion)
display(fig2)

# Plot of the body frame with respect to ECI frame
fig3 = PlotRecipe.framegif(time, ECI_frame, simdata.bodyframe, Tgif = 20, FPS = 8)
display(fig3)

# Plot of the animation of LVLH frame
# fig4 = PlotRecipe.frame_gif(time, simconfig.samplingtime, ECI_frame, orbitdata.LVLH, Tgif = 60, FPS = 8)
# display(fig4)
