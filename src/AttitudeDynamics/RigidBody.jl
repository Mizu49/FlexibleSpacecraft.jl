"""
    module RigidBody

module that consists variables and functions needed for the simulation of rigid body spacecraft attitude dynamics

# Usage
```
# Include module `RigidBody.jl`
include("RigidBody.jl")
using .RigidBody

"""
module RigidBody

using StaticArrays
using ..Frames

export update_angularvelocity, update_quaternion

"""
    struct RigidBodyModel

Struct of rigid body spacecraft model
"""
struct RigidBodyModel
    # Inertia Matrix
    inertia::Matrix
end

# Equation of dynamics
"""
    function _calc_differential_dynamics(model::RigidBodyModel, currentTime::Real, angularvelocity::SVector{3, <:Real}, current_body_frame::StaticArrays.SMatrix{3, 3, Real, 9}, disturbance::Vector{<:Real})

Get the differential of equation of dynamics. Internal function for module `RigidBody`

# Arguments
- model::RigidBodyModel
- currentTime: current time of system [s]
- angularvelocity: angular velocity of body frame with respect to ECI frame [rad/s]
- current_body_frame: current body frame [b1 b2 b3]

# return
- differential: differential of equation of motion
"""
function _calc_differential_dynamics(model::RigidBodyModel, currentTime::Real, angularvelocity::SVector{3, <:Real}, current_body_frame::StaticArrays.SMatrix{3, 3, <:Real, 9}, disturbance::Vector{<:Real})::SVector{3, <:Real}

    # skew matrix of angular velocity vector
    skewOmega = [
        0 -angularvelocity[3] angularvelocity[2]
        angularvelocity[3] 0 -angularvelocity[1]
        -angularvelocity[2] angularvelocity[1] 0]

    # calculate differential of equation of motion
    differential = SVector{3}(inv(model.inertia) * (disturbance - current_body_frame' * model.inertia * skewOmega * current_body_frame * current_body_frame' * angularvelocity))

    return differential
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
    function update_angularvelocity(model::RigidBodyModel, currentTime::Real, angularvelocity::Union{Vector{<:Real}, SVector{3, <:Real}}, Tsampling::Real, currentbodyframe::Frame, disturbance::Union{Vector{<:Real}, SVector{3, <:Real}})::SVector{3, <:Real}

calculate angular velocity at next time step using 4th order Runge-Kutta method
"""
function update_angularvelocity(model::RigidBodyModel, currentTime::Real, angularvelocity::Union{Vector{<:Real}, SVector{3, <:Real}}, Tsampling::Real, currentbodyframe::Frame, disturbance::Union{Vector{<:Real}, SVector{3, <:Real}})::SVector{3, <:Real}

    # define body frame matrix from struct `Frame`
    bodyframematrix = SMatrix{3, 3}(hcat(currentbodyframe.x, currentbodyframe.y, currentbodyframe.z))

    k1 = _calc_differential_dynamics(model, currentTime              , angularvelocity                   , bodyframematrix, disturbance)
    k2 = _calc_differential_dynamics(model, currentTime + Tsampling/2, angularvelocity + Tsampling/2 * k1, bodyframematrix, disturbance)
    k3 = _calc_differential_dynamics(model, currentTime + Tsampling/2, angularvelocity + Tsampling/2 * k2, bodyframematrix, disturbance)
    k4 = _calc_differential_dynamics(model, currentTime + Tsampling  , angularvelocity + Tsampling   * k3, bodyframematrix, disturbance)

    nextOmega = angularvelocity + Tsampling/6 * (k1 + 2*k2 + 2*k3 + k4)

    return nextOmega
end

# Update the quaternion vector (time evolution)
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

end
