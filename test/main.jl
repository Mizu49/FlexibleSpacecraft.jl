using Test, Profile

include("../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

# define parameter for the spacecraft
paramfilepath = "./test/spacecraft.yml"
(simconfig, attitudemodel, distconfig, initvalue, orbitinfo, strparam, strmodel, strdistconfig, attitudecontroller) = readparamfile(paramfilepath)

# run simulation
simtime = @timed (time, attitudedata, orbitdata, strdata) = runsimulation(attitudemodel, strmodel, initvalue, orbitinfo, distconfig, strdistconfig, simconfig, attitudecontroller)

@test quaternion_constraint(attitudedata.quaternion)

plottime = @timed begin # measure time for post process

    fig1 = PlotRecipe.angularvelocities(time, attitudedata.angularvelocity)
    fig2 = PlotRecipe.eulerangles(time, attitudedata.eulerangle)

    anim = PlotRecipe.framegif(time, T_UnitFrame2LVLHFrame * UnitFrame, attitudedata.RPYframe, Tgif = 5e-1, FPS = 20)

    # file output
    # location = "output" # specify where to save your data
    # outputdata = SimData(time, attitudedata, orbitdata)
    # write(location, outputdata)

    display(fig1)
    display(fig2)
    display(anim)

    if !isnothing(strmodel)
        plotlyjs()
        fig3 = plot(time, strdata.physicalstate[:, 1])
        fig3 = plot!(time, strdata.physicalstate[:, 2])
        display(fig3)
    end
end

println("Simulation time : $(simtime.time) (s)")
