using Test, Profile

include("../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

# define parameter for the spacecraft
paramfilepath = "./test/spacecraft.yml"
(simconfig, attitudemodel, distconfig, distinternals, initvalue, orbitinfo, orbitinternals, strparam, strmodel, strdistconfig, strinternals, attitudecontroller) = readparamfile(paramfilepath)

# run simulation
simtime = @timed simdata = runsimulation(attitudemodel, strmodel, initvalue, orbitinfo, orbitinternals, distconfig, distinternals, strdistconfig, strinternals, simconfig, attitudecontroller)

@test quaternion_constraint(simdata.attitude.quaternion)

plottime = @timed begin # measure time for post process

    fig1 = plot_angularvelocity(simdata.time, simdata.attitude.angularvelocity)
    fig2 = plot_eulerangles(simdata.time, simdata.attitude.eulerangle)

    anim = framegif(simdata.time, T_UnitFrame2LVLHFrame * UnitFrame, simdata.attitude.RPYframe, Tgif = 5e-1, FPS = 20)

    display(fig1)
    display(fig2)
    display(anim)

    if !isnothing(strmodel)
        fig3 = plot_physicalstate(simdata.time, simdata.appendages.physicalstate)
        display(fig3)
    end
end

println("Simulation time : $(simtime.time) (s)")
