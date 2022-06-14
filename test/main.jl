using Test

include("../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

# define parameter for the spacecraft
paramfilepath = "./test/spacecraft2.yml"
(simconfig, attitudemodel, distconfig, initvalue, orbitinfo, strparam, strmodel, strdistconfig) = readparamfile(paramfilepath)

# run simulation
println("Begin simulation!")
@time (time, attitudedata, orbitdata, strdata) = runsimulation(attitudemodel, strmodel, initvalue, orbitinfo, distconfig, strdistconfig, simconfig)
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
