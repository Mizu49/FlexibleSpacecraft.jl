using Test, Profile, GLMakie

include("../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

# define parameter for the spacecraft
paramfilepath = "./test/spacecraft2.yml"
(simconfig, attitudemodel, attidistinfo, initvalue, orbitinfo, appendageinfo, attitudecontroller) = readparamfile(paramfilepath)

# run simulation
simtime = @timed simdata = runsimulation(attitudemodel, initvalue, orbitinfo, attidistinfo, appendageinfo, simconfig, attitudecontroller)

# test the quaternion value to check the stability of the simulation
@test quaternion_constraint(simdata.attitude.quaternion)

fig1 = plot_angularvelocity(simdata.time, simdata.attitude.angularvelocity)
fig2 = plot_quaternion(simdata.time, simdata.attitude.quaternion)
fig3 = plot_eulerangles(simdata.time, simdata.attitude.eulerangle)

Makie.inline!(true)

display(fig1)
display(fig2)
display(fig3)

if !isnothing(appendageinfo.model)
    fig4 = plot_physicalstate(simdata.time, simdata.appendages.physicalstate)
    display(fig4)
end

fig5 = plot_angular_momentum(simdata.time, simdata.attitude.angularmomentum)
display(fig5)

# spacecraft attitude animation
# animate_attitude(simdata.time, simdata.attitude.eulerangle)

println("Simulation time : $(simtime.time) (s)")
