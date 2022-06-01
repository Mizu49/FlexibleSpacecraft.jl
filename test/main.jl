using Test

include("../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

# Set the dynamics model
attitudemodel = setdynamicsmodel("./test/spacecraft2.yml",)

# define a orbit info
orbitinfo = setorbit("./test/orbit2.yml", ECI_frame)

# define parameter and simulation model for the flexible appendages
(strparam, strmodel) = setstructure("./test/module-tests/param-springmass.yml")

# Set disturbance torque
distconfig = setdisturbance("./test/disturbance.yml")

# Initialize the simulation configuration
simconfig = setsimconfig("./test/simconfig.yml")

# Define initial values for simulation
initvalue = setinitvalue("./test/initvalue.yml")

# run simulation
println("Begin simulation!")
@time (time, attitudedata, orbitdata, strdata) = runsimulation(attitudemodel, strmodel, initvalue, orbitinfo, distconfig, simconfig)
println("Completed!")

@test quaternion_constraint(attitudedata.quaternion)

fig1 = PlotRecipe.angularvelocities(time, attitudedata.angularvelocity)
display(fig1)

fig2 = PlotRecipe.quaternions(time, attitudedata.quaternion)
display(fig2)

# Plot of the body frame with respect to ECI frame
fig3 = PlotRecipe.framegif(time, LVLHref, attitudedata.RPYframe, Tgif = 20, FPS = 8)
display(fig3)

# Plot of the euler angle
fig4 = PlotRecipe.eulerangles(time, attitudedata.eulerangle)
display(fig4)

fig5 = plot(time, strdata.physicalstate[:, 1])
fig5 = plot!(time, strdata.physicalstate[:, 2])
display(fig5)

location = "output" # specify where to save your data
outputdata = SimData(time, attitudedata, orbitdata)
write(location, outputdata)
