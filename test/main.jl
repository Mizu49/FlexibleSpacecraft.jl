using Test, Profile

include("../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

# define parameter for the spacecraft
paramfilepath = "./test/spacecraft2.yml"
(simconfig, attitudemodel, distconfig, distinternals, initvalue, orbitinfo, strparam, strmodel, strdistconfig, strinternals, attitudecontroller) = readparamfile(paramfilepath)

# run simulation
simtime = @timed simdata = runsimulation(attitudemodel, strmodel, initvalue, orbitinfo, distconfig, distinternals, strdistconfig, strinternals, simconfig, attitudecontroller)

# test the quaternion value to check the stability of the simulation
@test quaternion_constraint(simdata.attitude.quaternion)

fig1 = plot_angularvelocity(simdata.time, simdata.attitude.angularvelocity)
fig2 = plot_quaternion(simdata.time, simdata.attitude.quaternion)
fig3 = plot_eulerangles(simdata.time, simdata.attitude.eulerangle)

anim = framegif(simdata.time, T_UnitFrame2LVLHFrame * UnitFrame, simdata.attitude.RPYframe, Tgif = 1e-1, FPS = 20)

display(fig1)
display(fig2)
display(fig3)
display(anim)

if !isnothing(strmodel)
    fig4 = plot_physicalstate(simdata.time, simdata.appendages.physicalstate)
    display(fig4)
end

fig5 = plot_angular_momentum(simdata.time, simdata.attitude.angularmomentum)
display(fig5)

println("Simulation time : $(simtime.time) (s)")
