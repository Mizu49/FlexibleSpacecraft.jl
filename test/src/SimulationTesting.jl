module SimulationTesting

include("../../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft
using Test
using StructArrays
using StaticArrays

"""
    function quaternion_constraint(quaternion::Vector{StaticArrays.SVector{4, Float64}})

Check if quaternion vector satisfies constraint
"""
function quaternion_constraint(quaternion::Vector{StaticArrays.SVector{4, Float64}})

    datanum = size(quaternion, 1)

    # initialize BitArray for evaluation
    logicalArray = falses(datanum)

    # evaluate constraint of quaiteratively
    for cnt = 1:datanum
        constraint = quaternion[cnt, 1]^2 + quaternion[cnt, 2]^2 + quaternion[cnt, 3]^2 + quaternion[cnt, 4]^2

        if constraint ≈ 1 rtol = 1e-9
            logicalArray[cnt] = true
        else
            logicalArray[cnt] = false
        end
    end

    return all(logicalArray)
end

end
