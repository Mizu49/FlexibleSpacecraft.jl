using Test

include("../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

# Sampling period of simulation (second)
Tsampling = 1e-2
# Time length of simulation (second)
simulation_time = 1000

# Set the dynamics model
model = setdynamicsmodel("./test/spacecraft.yml",)

# define a orbit info
orbitinfo = initorbitinfo("./test/orbit2.yml", ECI_frame)

# Set disturbance torque
distconfig = DisturbanceConfig(constanttorque = [0.05, 0.0, 0])

# Initialize the simulation configuration
simconfig = setsimconfig(simulation_time, Tsampling)

# Define initial values for simulation
initvalue = setinitvalue()

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
fig3 = PlotRecipe.framegif(time, LVLHref, simdata.rollpitchyawframe, Tgif = 20, FPS = 8)
display(fig3)

# Plot of the euler angle
fig4 = PlotRecipe.eulerangles(time, simdata.eulerangle)
display(fig4)
