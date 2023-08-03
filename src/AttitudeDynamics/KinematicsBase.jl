"""
    module KinematicsBase

module for functions and APIs for the attitude kinematics
"""
module KinematicsBase

using StaticArrays
using ..Frames

export InitKinematicsData, AttitudeData, initattitudedata, update_quaternion

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
    struct InitKinematicsData

Struct that consists of the initial state value of the time-variant physical amounts in simulation
"""
struct InitKinematicsData

    # Spacecraft state variable
    quaternion::SVector{4, Real}
    angularvelocity::SVector{3, Real}
    bodyframe::Frame

end

struct AttitudeData
    datanum::Int
    quaternion::Vector{SVector{4, <:Real}}
    angularvelocity::Vector{SVector{3, <:Real}}
    angularmomentum::Vector{SVector{3, <:Real}}
    C_ECI2BRF::Vector{SMatrix{3, 3}}
    RPYframe::Vector{<:Frame}
    eulerangle::Vector{SVector{3, <:Real}}
end

"""
    initattitudedata(datanum::Int, initialdata::InitKinematicsData)

Initialize the data container for the attitude dynamics
"""
function initattitudedata(datanum::Int, initialdata::InitKinematicsData)

    return AttitudeData(
        datanum,
        _initquaternion(datanum, initialdata.quaternion),
        _initangularvelocity(datanum, initialdata.angularvelocity),
        [SVector{3}(0.0, 0.0, 0.0) for _ in 1:datanum],
        [SMatrix{3, 3}(zeros(3, 3)) for _ = 1:datanum],
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

end
