using Test

include("../../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft
using StaticArrays

# parameter setting
paramfile = "./test/module-tests/param-springmass.yml"
(params, model) = set_appendages_model(paramfile)

# Test the simulation feature
Ts = 1e-4
times = 0:Ts:10

datanum = size(times, 1)

simdata = initappendagedata(model, [0, 0, 0, 0], datanum)

@time for cnt = 1:datanum
    # Calculate the state in physical coordinate
    simdata.physicalstate[cnt] = modalstate2physicalstate(model, simdata.state[cnt])

    ctrlinput = 1 * sin(5 * times[cnt])
    distinput = 5 * sin(10* times[cnt])

    if cnt != datanum
        # Update the state and proceed to the next computation
        simdata.state[cnt+1] = updatestate(model, Ts, times[cnt], simdata.state[cnt], zeros(3), ctrlinput, distinput)
    end

    simdata.controlinput[cnt] = ctrlinput
    simdata.disturbance[cnt] = distinput
end

plotlyjs()
fig1 = plot(times, simdata.physicalstate[:, 1])
fig1 = plot!(times, simdata.physicalstate[:, 2])

fig2 = plot(times, simdata.controlinput[:, 1], label = "control input")
fig2 = plot!(times, simdata.disturbance[:, 1], label = "disturbance")

display(fig1)
display(fig2)
