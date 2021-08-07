module SimulationTesting

include("../../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft
using Test

"""
    function quaternion_constraint(quaternion)

Check if quaternion vector satisfies constraint
"""
function quaternion_constraint(quaternion)

    datanum = size(quaternion, 2)

    # initialize BitArray for evaluation
    logicalArray = falses(datanum)

    # evaluate constraint of quaiteratively
    for cnt = 1:datanum
        constraint = quaternion[1, cnt]^2 + quaternion[2, cnt]^2 + quaternion[3, cnt]^2 + quaternion[4, cnt]^2

        if constraint ≈ 1 rtol = 1e-9
            logicalArray[cnt] = true
        else
            logicalArray[cnt] = false
        end
    end

    return all(logicalArray)
end

end
