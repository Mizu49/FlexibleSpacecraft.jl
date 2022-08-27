"""
    module Evaluation

Functions and APIs for the evaluation of the simulation result. Checking the constraints and the necessary evaluation on the simulation.
"""
module Evaluation

using StaticArrays

export quaternion_constraint

"""
    function quaternion_constraint(quaternion::Vector{SVector{4, <:Real}})

Check if quaternion vector satisfies constraint
"""
function quaternion_constraint(quaternion::Vector{SVector{4, <:Real}})

    datanum = size(quaternion, 1)

    # initialize BitArray for evaluation
    logicalArray = falses(datanum)

    # evaluate constraint of quaiteratively
    for cnt = 1:datanum

        q = quaternion[cnt]

        constraint = q[1]^2 + q[2]^2 + q[3]^2 + q[4]^2

        if constraint ≈ 1 rtol = 1e-9
            logicalArray[cnt] = true
        else
            logicalArray[cnt] = false
        end
    end

    return all(logicalArray)
end

end
