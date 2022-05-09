using Test

include("../../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft
using Plots, StaticArrays

# parameter setting
paramfile = "./test/module-tests/param-springmass.yml"
(params, model) = setstructure(paramfile)

# Test the simulation feature
Ts = 1e-4
times = 0:Ts:10

datanum = size(times, 1)

simdata = initdatacontainer(model, [1e-3, 0, 0, 0], datanum)

@time for cnt = 1:datanum
    # Calculate the state in physical coordinate
    simdata.physicalstate[cnt] = modalstate2physicalstate(model, simdata.state[cnt])

    if cnt != datanum
        # Update the state and proceed to the next computation
        simdata.state[cnt+1] = updatestate(model, Ts, times[cnt], simdata.state[cnt], zeros(3), 0, 0)
    end
end

plotlyjs()
fig = plot(times, simdata.physicalstate[:, 1])
fig = plot!(times, simdata.physicalstate[:, 2])
display(fig)
