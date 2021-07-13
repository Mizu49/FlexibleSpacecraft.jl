"""
    module RigidBodyAttitudeDynamics

module that consists variables and functions needed for the simulation of rigid body spacecraft attitude dynamics

# Usage
```
# Include module `RigidBodyAttitudeDynamics.jl`
include("RigidBodyAttitudeDynamics.jl")
using .RigidBodyAttitudeDynamics

"""
module RigidBodyAttitudeDynamics


"""
    DynamicsModel(inertia::Matrix)

mutable struct of attitude dynamics model
- inertia: inertia matrix of a given system
"""
mutable struct DynamicsModel
    # Inertia Matrix
    inertia::Matrix
end

# Equation of dynamics
"""
    calc_differential_dynamics(model::DynamicsModel, currentTime, currentOmega, currentCoordB)

Get the differential of equation of dynamics.

# Arguments
- model::DynamicsModel
- currentTime: current time of system [s]
- currentOmega: angular velocity of system [rad/s]
- currentCoordB: current coordinate matrix [b1 b2 b3]

# return
- differential: differential of equation of motion
"""
function calc_differential_dynamics(model::DynamicsModel, currentTime, currentOmega, currentCoordB, disturbance)

    # skew matrix of angular velocity vector
    skewOmega = [
        0 -currentOmega[3] currentOmega[2]
        currentOmega[3] 0 -currentOmega[1]
        -currentOmega[2] currentOmega[1] 0]

    # calculate differential of equation of motion
    differential = inv(model.inertia) * (disturbance - currentCoordB' * model.inertia * skewOmega * currentCoordB * currentCoordB' * currentOmega)

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
function calc_differential_kinematics(omega, quaternion)

    OMEGA = [
        0 omega[3] -omega[2] omega[1]
        -omega[3] 0 omega[1] omega[2]
        omega[2] -omega[1] 0 omega[3]
        -omega[1] -omega[2] -omega[3] 0
    ]

    differential = 1/2 * OMEGA * quaternion

    return differential
end

"""
    update_angular_velocity(model::DynamicsModel, currentTime, currentOmega, samplingTime, currentCoordB)

calculate angular velocity at next time step using 4th order Runge-Kutta method
"""
function calc_angular_velocity(model::DynamicsModel, currentTime, currentOmega, samplingTime, currentCoordB, disturbance)
    # Update the angular velocity vector using 4th order runge kutta method

    k1 = calc_differential_dynamics(model, currentTime                 , currentOmega                      , currentCoordB, disturbance)
    k2 = calc_differential_dynamics(model, currentTime + samplingTime/2, currentOmega + samplingTime/2 * k1, currentCoordB, disturbance)
    k3 = calc_differential_dynamics(model, currentTime + samplingTime/2, currentOmega + samplingTime/2 * k2, currentCoordB, disturbance)
    k4 = calc_differential_dynamics(model, currentTime + samplingTime  , currentOmega + samplingTime   * k3, currentCoordB, disturbance)

    nextOmega = currentOmega + samplingTime/6 * (k1 + 2*k2 + 2*k3 + k4)

    return nextOmega
end


# Update the quaternion vector (time evolution)
"""
    update_quaternion(currentOmega, currentQuaternion, samplingTime)

calculate quaternion at next time step using 4th order Runge-Kutta method.
"""
function calc_quaternion(currentOmega, currentQuaternion, samplingTime)
    # Update the quaterion vector using 4th order runge kutta method

    k1 = calc_differential_kinematics(currentOmega, currentQuaternion                      );
    k2 = calc_differential_kinematics(currentOmega, currentQuaternion + samplingTime/2 * k1);
    k3 = calc_differential_kinematics(currentOmega, currentQuaternion + samplingTime/2 * k2);
    k4 = calc_differential_kinematics(currentOmega, currentQuaternion + samplingTime   * k3);

    nextQuaternion = currentQuaternion + samplingTime/6 * (k1 + 2*k2 + 2*k3 + k4);

    return nextQuaternion
end

"""
    calc_transformation_matrix(q)

Calculate the transformation matrix from coordinate A system (inertial frame) to coordinate B system (spacecraft body fixed frame).

# Arguments
- `q`: quaternion

# Return
- `transformation_matrix`: transformation matrix from referential frame to body fixed frame
"""
function calc_transformation_matrix(q)

    # Check if the quaterion satisfies its constraint
    try
        constraint = q[1]^2 + q[2]^2 + q[3]^2 + q[4]^2

    catch constraint

        if constraint < 0.995
            error("Quaternion does not satisfy constraint")
        elseif constraint > 1.005
            error("Quaternion does not satisfy constraint")
        end
    end

    transformation_matrix = [
        q[1]^2 - q[2]^2 - q[3]^2 + q[4]^2  2*(q[1]*q[2] + q[3]*q[4])          2*(q[1]*q[3] - q[2]*q[4])
        2*(q[2]*q[1] - q[3]*q[4])          q[2]^2 - q[3]^2 - q[1]^2 + q[4]^2  2*(q[2]*q[3] + q[1]*q[4])
        2*(q[3]*q[1] + q[2]*q[4])          2*(q[3]*q[2] - q[1]*q[4])          q[3]^2 - q[1]^2 - q[2]^2 + q[4]^2
    ]

    return transformation_matrix
end


end
