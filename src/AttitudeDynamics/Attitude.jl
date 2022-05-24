"""
    module Attitude

module for functions and APIs for the attitude kinematics
"""
module Attitude

using StaticArrays
using ..Frames

export InitData, AttitudeData, initattitudedata, update_quaternion, dcm2quaternion, euler2dcm, quaternion2dcm, dcm2euler, quaternion2euler, euler2quaternion

"""
    function _initangularvelocity(simdata_num, initital_value::Vector)

Initialize array that contains time response of angular velocity
"""
function _initangularvelocity(datanum, initialvelocity::SVector{3, <:Real})

    angularvelocitydata = [SVector(0.0, 0.0, 0.0) for _ in 1:datanum]
    angularvelocitydata[1] = initialvelocity

    return angularvelocitydata
end

"""
    function _initquaternion(simdata_num, initial_value::Vector[4])

initialize array that contains time response of quaternion
"""
function _initquaternion(datanum, initialvalue::SVector{4, <:Real})

    quaterniondata = [SVector(0.0, 0.0, 0.0, 0.0) for _ in 1:datanum]
    quaterniondata[1] = initialvalue

    return quaterniondata
end

"""
    struct InitData

Struct that consists of the initial state value of the time-variant physical amounts in simulation
"""
struct InitData

    # Spacecraft state variable
    quaternion::SVector{4, Real}
    angularvelocity::SVector{3, Real}
    bodyframe::Frame

end

struct AttitudeData
    datanum::Int
    quaternion::Vector{SVector{4, <:Real}}
    angularvelocity::Vector{SVector{3, <:Real}}
    bodyframe::Vector{<:Frame}
    RPYframe::Vector{<:Frame}
    eulerangle::Vector{SVector{3, <:Real}}
end

"""
    initattitudedata(datanum::Int, initialdata::InitData)

Initialize the data container for the attitude dynamics
"""
function initattitudedata(datanum::Int, initialdata::InitData)

    return AttitudeData(
        datanum,
        _initquaternion(datanum, initialdata.quaternion),
        _initangularvelocity(datanum, initialdata.angularvelocity),
        initframes(datanum, initialdata.bodyframe),
        initframes(datanum, initialdata.bodyframe),
        [SVector{3}(0.0, 0.0, 0.0) for _ in 1:datanum]
    )
end

"""
    function update_quaternion(angularvelocity, quaternion, Tsampling)::SVector{4, <:Real}

calculate quaternion at next time step using 4th order Runge-Kutta method.
"""
function update_quaternion(angularvelocity, quaternion, Tsampling)::SVector{4, <:Real}
    # Update the quaterion vector using 4th order runge kutta method

    k1 = _calcdifferential_kinematics(angularvelocity, quaternion                   );
    k2 = _calcdifferential_kinematics(angularvelocity, quaternion + Tsampling/2 * k1);
    k3 = _calcdifferential_kinematics(angularvelocity, quaternion + Tsampling/2 * k2);
    k4 = _calcdifferential_kinematics(angularvelocity, quaternion + Tsampling   * k3);

    nextQuaternion = quaternion + Tsampling/6 * (k1 + 2*k2 + 2*k3 + k4);

    return nextQuaternion
end

# Equation of Quaternion
"""
    _calcdifferential_kinematics(omega::Vector, quaterion::Vector)

Get differential of quaternion from equation of kinematics

# Arguments
- omega: angular velocity of system
- quaterion: current value of quaternion

# Return
- differential: differential of equation of kinematics
"""
function _calcdifferential_kinematics(angularvelocity::SVector{3, <:Real}, quaternion::SVector{4, <:Real})::SVector{4, <:Real}

    OMEGA = [
        0 angularvelocity[3] -angularvelocity[2] angularvelocity[1]
        -angularvelocity[3] 0 angularvelocity[1] angularvelocity[2]
        angularvelocity[2] -angularvelocity[1] 0 angularvelocity[3]
        -angularvelocity[1] -angularvelocity[2] -angularvelocity[3] 0
    ]

    differential = SVector{4}(1/2 * OMEGA * quaternion)

    return differential
end

"""
    dcm2quaternion(dcm::Matrix{Real})::Vector{Real}

calculate quaternion from direction cosine matrix (DCM) `dcm`
"""
function dcm2quaternion(dcm::Union{SMatrix{3, 3, <:Real}, Matrix{<:Real}})::SVector{4, Real}

    _checkdcm(dcm)

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

"""
    function eular2dcm(euler::Union{SVector{3, <:Real}, Vector{<:Real}})::SMatrix{3, 3, <:Real}

calculate direction cosine matrix from the vector of z-y-x eular angles.

## Argument

* `euler::Union{SVector{3, <:Real}, Vector{<:Real}}`: each element represents the rotation with z, y, x axis, respectively
"""
function euler2dcm(euler::Union{SVector{3, <:Real}, Vector{<:Real}})::SMatrix{3, 3, <:Real}
    s1 = sin(euler[1])
    s2 = sin(euler[2])
    s3 = sin(euler[3])
    c1 = cos(euler[1])
    c2 = cos(euler[2])
    c3 = cos(euler[3])

    dcm = SMatrix{3, 3}([
        c2*c3 c2*s3 -s2;
        (-c1*s3 + s1*s2*c3) (c1*c3 + s1*s2*s3) s1*c2;
        (s1*s3 + c1*s2*c3) (-s1*c3 + c1*s2*s3) c1*c2
    ])
end

"""
    quaternion2dcm(q::Union{Vector{<:Real}, SVector{4, <:Real}})

calculates direction cosine matrix from quaternion
"""
function quaternion2dcm(q::Union{Vector{<:Real}, SVector{4, <:Real}})::SMatrix{3, 3, <:Real}
    q2 = q.^2;

    dcm = SMatrix{3, 3}([
        (q2[1] - q2[2] - q2[3] + q2[4]) 2*(q[1]*q[2] + q[3]*q[4]) 2*(q[1]*q[3] - q[2]*q[4]);
        2*(q[1]*q[2] - q[3]*q[4]) (q2[2] - q2[1] - q2[3] + q2[4]) 2*(q[2]*q[3] + q[1]*q[4]);
        2*(q[1]*q[3] + q[2]*q[4]) 2*(q[2]*q[3] - q[1]*q[4]) q2[3] - q2[1] - q2[2] + q2[4]
    ])

    return dcm
end

"""
    dcm2euler(dcm::Union{SMatrix{3, 3, <:Real}, Matrix{<:Real}})::SVector{3, <:Real}

calculates z-y-x euler rotation angle from direction cosine matrix
"""
function dcm2euler(dcm::Union{SMatrix{3, 3, <:Real}, Matrix{<:Real}})::SVector{3, <:Real}
    _checkdcm(dcm)

    euler = SVector{3}([
        atan(dcm[2,3], dcm[3,3]),
        atan(-dcm[1,3], sqrt(dcm[2,3]^2 + dcm[3,3]^2)),
        atan(dcm[1,2], dcm[1,1])
    ])

    return euler
end

"""
    quaternion2euler(quaternion::Union{Vector{<:Real}, SVector{4, <:Real}})::SVector{3, <:Real}

calculates z-y-x euler rotation angle from quaternion
"""
function quaternion2euler(quaternion::Union{Vector{<:Real}, SVector{4, <:Real}})::SVector{3, <:Real}

    # use DCM for the calculation
    euler = dcm2euler(quaternion2dcm(quaternion))

    return euler
end

"""
    euler2quaternion(euler::Union{SVector{3, <:Real}, Vector{<:Real}})::SVector{4, Real}

calculates quaternion from z-y-x euler rotation angle
"""
function euler2quaternion(euler::Union{SVector{3, <:Real}, Vector{<:Real}})::SVector{4, Real}

    # use DCM for the calculation
    quaternion = dcm2quaternion(euler2dcm(euler))

    return quaternion
end

function _checkdcm(dcm::Union{SMatrix{3, 3, <:Real}, Matrix{<:Real}})
    if size(dcm) != (3, 3)
        throw(ArgumentError("`dcm` should be `3x3` matrix"))
    end
    return
end

"""
    Base.Math.deg2rad(rotationangle::Union{SVector{3, <:Real}, Vector{<:Real})

Convert rotation angle vector in degrees to radians
"""
function Base.Math.deg2rad(rotationangle::Union{SVector{3, <:Real}, Vector{<:Real}})

    newangle = SVector{3}([
        deg2rad(rotationangle[1])
        deg2rad(rotationangle[2])
        deg2rad(rotationangle[3])
    ])

    return newangle
end

"""
    Base.Math.rad2deg(rotationangle::Union{SVector{3, <:Real}, Vector{<:Real})

Convert rotation angle vector in radians to degrees
"""
function Base.Math.rad2deg(rotationangle::Union{SVector{3, <:Real}, Vector{<:Real}})

    newangle = SVector{3}([
        rad2deg(rotationangle[1])
        rad2deg(rotationangle[2])
        rad2deg(rotationangle[3])
    ])

    return newangle
end


end
