using Test, Profile, GLMakie

include("../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

# define parameter for the spacecraft
include("spacecraft1.jl")
(simconfig, attitudemodel, attidistinfo, initvalue, orbitinfo, appendageinfo, attitudecontroller) = set_simulation_parameters(spacecraft)

# run simulation
simtime = @timed simdata = runsimulation(attitudemodel, initvalue, orbitinfo, attidistinfo, appendageinfo, simconfig, attitudecontroller)

# test the quaternion value to check the stability of the simulation
@test quaternion_constraint(simdata.attitude.quaternion)

fig_angularvelocity = plot_angularvelocity(simdata.time, simdata.attitude.angularvelocity)
fig_quaternion = plot_quaternion(simdata.time, simdata.attitude.quaternion)
fig_rollpitchyaw = plot_eulerangles(simdata.time, simdata.attitude.eulerangle)

Makie.inline!(true)

display(fig_angularvelocity)
display(fig_quaternion)
display(fig_rollpitchyaw)

if !isnothing(appendageinfo)
    fig_appendage_displacement = plot_physicalstate(simdata.time, simdata.appendages.physicalstate)
    display(fig_appendage_displacement)
end

fig_angularmomentum = plot_angular_momentum(simdata.time, simdata.attitude.angularmomentum)
display(fig_angularmomentum)

# spacecraft attitude animation
# animate_attitude(simdata.time, simdata.attitude.eulerangle)
