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

physicalstate = [zeros(SVector{4}) for _ in 1:datanum]
physicalstate[1] = SVector{4}([1e-3, 0, 0, 0])
state = [zeros(SVector{4}) for _ in 1:datanum]
state[1] = physicalstate2modalstate(model, physicalstate[1])

@time for cnt = 1:datanum
    # Calculate the state in physical coordinate
    physicalstate[cnt] = modalstate2physicalstate(model, state[cnt])

    if cnt != datanum
        # Update the state and proceed to the next computation
        state[cnt+1] = updatestate(model, Ts, times[cnt], state[cnt], zeros(3), 0, 0)
    end
end

plotlyjs()
fig = plot(times, physicalstate[:, 1])
fig = plot!(times, physicalstate[:, 2])
display(fig)
