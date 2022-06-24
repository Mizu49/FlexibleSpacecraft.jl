using Test, Profile

include("../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

# define parameter for the spacecraft
paramfilepath = "./test/spacecraft.yml"
(simconfig, attitudemodel, distconfig, initvalue, orbitinfo, strparam, strmodel, strdistconfig) = readparamfile(paramfilepath)

# run simulation
simtime = @timed (time, attitudedata, orbitdata, strdata) = runsimulation(attitudemodel, strmodel, initvalue, orbitinfo, distconfig, strdistconfig, simconfig)

@test quaternion_constraint(attitudedata.quaternion)

plottime = @timed begin # measure time for post process

    fig1 = PlotRecipe.angularvelocities(time, attitudedata.angularvelocity)
    fig2 = PlotRecipe.quaternions(time, attitudedata.quaternion)
    fig4 = PlotRecipe.eulerangles(time, attitudedata.eulerangle)
    fig5 = plot(time, strdata.physicalstate[:, 1])
    fig5 = plot!(time, strdata.physicalstate[:, 2])

    location = "output" # specify where to save your data
    outputdata = SimData(time, attitudedata, orbitdata)
    write(location, outputdata)

    display(fig1)
    display(fig2)
    display(fig4)
    display(fig5)
end

println("Simulation time : $(simtime.time) (s)")
