"""
    module Attitude

module for functions and APIs for the attitude kinematics
"""
module Attitude

using StaticArrays

export dcm2quaternion

"""
    dcm2quaternion(dcm::Matrix{Real})::Vector{Real}

calculate quaternion from direction cosine matrix (DCM) `dcm`
"""
function dcm2quaternion(dcm::Union{SMatrix{3, 3, <:Real}, Matrix{<:Real}})::SVector{4, Real}

    checkdcm(dcm)

    q = [
        sqrt(1 + dcm[1,1] - dcm[2,2] - dcm[3,3])/2,
        sqrt(1 - dcm[1,1] + dcm[2,2] - dcm[3,3])/2,
        sqrt(1 - dcm[1,1] - dcm[2,2] + dcm[3,3])/2,
        sqrt(1 + dcm[1,1] + dcm[2,2] + dcm[3,3])/2
    ]

    (maxvalue, maxindex) = findmax(q)

    if maxindex == 1
        q[2] = 0.25/q[1] * (dcm[1,2] + dcm[2,1])
        q[3] = 0.25/q[1] * (dcm[1,3] + dcm[3,1])
        q[4] = 0.25/q[1] * (dcm[2,3] - dcm[3,2])
    elseif maxindex == 2
        q[1] = 0.25/q[2] * (dcm[1,2] + dcm[2,1])
        q[3] = 0.25/q[2] * (dcm[3,2] + dcm[2,3])
        q[4] = 0.25/q[2] * (dcm[3,1] - dcm[1,3])
    elseif maxindex == 3
        q[1] = 0.25/q[3] * (dcm[3,1] + dcm[1,3])
        q[2] = 0.25/q[3] * (dcm[3,2] + dcm[2,3])
        q[4] = 0.25/q[3] * (dcm[1,2] - dcm[2,1])
    elseif maxindex == 4
        q[1] = 0.25/q[4] * (dcm[2,3] - dcm[3,2])
        q[2] = 0.25/q[4] * (dcm[3,1] - dcm[1,3])
        q[3] = 0.25/q[4] * (dcm[1,2] - dcm[2,1])
    else
        error("`maxindex` is illegal")
    end

    return SVector{4}(q)
end

function checkdcm(dcm::Union{SMatrix{3, 3, <:Real}, Matrix{<:Real}})
    if size(dcm) != (3, 3)
        throw(ArgumentError("`dcm` should be `3x3` matrix"))
    end
    return
end

end
