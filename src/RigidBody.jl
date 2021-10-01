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

using ..Frames

export calc_angular_velocity, calc_quaternion

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
    calc_differential_dynamics(model::RigidBodyModel, currentTime, angular_velocity, current_body_frame)

Get the differential of equation of dynamics.

# Arguments
- model::RigidBodyModel
- currentTime: current time of system [s]
- angular_velocity: angular velocity of body frame with respect to ECI frame [rad/s]
- current_body_frame: current body frame [b1 b2 b3]

# return
- differential: differential of equation of motion
"""
function calc_differential_dynamics(model::RigidBodyModel, currentTime, angular_velocity, current_body_frame, disturbance)

    # skew matrix of angular velocity vector
    skewOmega = [
        0 -angular_velocity[3] angular_velocity[2]
        angular_velocity[3] 0 -angular_velocity[1]
        -angular_velocity[2] angular_velocity[1] 0]

    # calculate differential of equation of motion
    differential = inv(model.inertia) * (disturbance - current_body_frame' * model.inertia * skewOmega * current_body_frame * current_body_frame' * angular_velocity)

    return differential
end


# Equation of Quaternion
"""
    calc_differential_kinematics(omega::Vector, quaterion::Vector)

Get differential of quaternion from equation of kinematics

# Arguments
- omega: angular velocity of system
- quaterion: current value of quaternion

# Return
- differential: differential of equation of kinematics
"""
function calc_differential_kinematics(angular_velocity, quaternion)

    OMEGA = [
        0 angular_velocity[3] -angular_velocity[2] angular_velocity[1]
        -angular_velocity[3] 0 angular_velocity[1] angular_velocity[2]
        angular_velocity[2] -angular_velocity[1] 0 angular_velocity[3]
        -angular_velocity[1] -angular_velocity[2] -angular_velocity[3] 0
    ]

    differential = 1/2 * OMEGA * quaternion

    return differential
end

"""
    function calc_angular_velocity(model::RigidBodyModel, currentTime, angular_velocity::Vector, Tsampling, currentbodyframe::Frame, disturbance::Vector)

calculate angular velocity at next time step using 4th order Runge-Kutta method
"""
function calc_angular_velocity(model::RigidBodyModel, currentTime, angular_velocity::Vector, Tsampling, currentbodyframe::Frame, disturbance::Vector)

    # define body frame matrix from struct `Frame`
    bodyframematrix = hcat(currentbodyframe.x, currentbodyframe.y, currentbodyframe.z)

    k1 = calc_differential_dynamics(model, currentTime              , angular_velocity                   , bodyframematrix, disturbance)
    k2 = calc_differential_dynamics(model, currentTime + Tsampling/2, angular_velocity + Tsampling/2 * k1, bodyframematrix, disturbance)
    k3 = calc_differential_dynamics(model, currentTime + Tsampling/2, angular_velocity + Tsampling/2 * k2, bodyframematrix, disturbance)
    k4 = calc_differential_dynamics(model, currentTime + Tsampling  , angular_velocity + Tsampling   * k3, bodyframematrix, disturbance)

    nextOmega = angular_velocity + Tsampling/6 * (k1 + 2*k2 + 2*k3 + k4)

    return nextOmega
end

# Update the quaternion vector (time evolution)
"""
    update_quaternion(angular_velocity, currentQuaternion, Tsampling)

calculate quaternion at next time step using 4th order Runge-Kutta method.
"""
function calc_quaternion(angular_velocity, quaternion, Tsampling)
    # Update the quaterion vector using 4th order runge kutta method

    k1 = calc_differential_kinematics(angular_velocity, quaternion                   );
    k2 = calc_differential_kinematics(angular_velocity, quaternion + Tsampling/2 * k1);
    k3 = calc_differential_kinematics(angular_velocity, quaternion + Tsampling/2 * k2);
    k4 = calc_differential_kinematics(angular_velocity, quaternion + Tsampling   * k3);

    nextQuaternion = quaternion + Tsampling/6 * (k1 + 2*k2 + 2*k3 + k4);

    return nextQuaternion
end

end
