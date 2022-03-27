using Test

include("../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

# Set the dynamics model
model = setdynamicsmodel("./test/spacecraft.yml",)

# define a orbit info
orbitinfo = setorbit("./test/orbit2.yml", ECI_frame)

# Set disturbance torque
distconfig = setdisturbance("./test/disturbance.yml")

# Initialize the simulation configuration
simconfig = setsimconfig("./test/simconfig.yml")

# Define initial values for simulation
initvalue = setinitvalue("./test/initvalue.yml")

# run simulation
println("Begin simulation!")
@time (time, attitudedata, orbitdata) = runsimulation(model, initvalue, orbitinfo, distconfig, simconfig)
println("Completed!")

@test Evaluation.quaternion_constraint(attitudedata.quaternion)

# fig1 = PlotRecipe.angularvelocities(time, attitudedata.angularvelocity)
# # fig1 = PlotRecipe.angularvelocities(time, attitudedata.angularvelocity, timerange = (0, 10))
# display(fig1)

# fig2 = PlotRecipe.quaternions(time, attitudedata.quaternion)
# display(fig2)

# # Plot of the body frame with respect to ECI frame
# fig3 = PlotRecipe.framegif(time, LVLHref, attitudedata.rollpitchyawframe, Tgif = 20, FPS = 8)
# display(fig3)

# # Plot of the euler angle
# fig4 = PlotRecipe.eulerangles(time, attitudedata.eulerangle)
# display(fig4)

outputdata = SimData(time, attitudedata, orbitdata)
write("output", outputdata)
