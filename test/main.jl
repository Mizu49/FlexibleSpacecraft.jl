using Test, Profile

include("../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

# define parameter for the spacecraft
paramfilepath = "./test/spacecraft.yml"
(simconfig, attitudemodel, distconfig, distinternals, initvalue, orbitinfo, strparam, strmodel, strdistconfig, strinternals, attitudecontroller) = readparamfile(paramfilepath)

# run simulation
simtime = @timed simdata = runsimulation(attitudemodel, strmodel, initvalue, orbitinfo, distconfig, distinternals, strdistconfig, strinternals, simconfig, attitudecontroller)

@test quaternion_constraint(simdata.attitude.quaternion)

plottime = @timed begin # measure time for post process

    fig1 = PlotRecipe.angularvelocities(simdata.time, simdata.attitude.angularvelocity)
    fig2 = PlotRecipe.eulerangles(simdata.time, simdata.attitude.eulerangle)

    anim = PlotRecipe.framegif(simdata.time, T_UnitFrame2LVLHFrame * UnitFrame, simdata.attitude.RPYframe, Tgif = 5e-1, FPS = 20)

    display(fig1)
    display(fig2)
    display(anim)

    if !isnothing(strmodel)
        plotlyjs()
        fig3 = plot(simdata.time, simdata.appendages.physicalstate[:, 1])
        fig3 = plot!(simdata.time, simdata.appendages.physicalstate[:, 2])
        display(fig3)
    end
end

println("Simulation time : $(simtime.time) (s)")
