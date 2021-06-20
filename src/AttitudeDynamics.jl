"""
    module AttitudeDynamics

module that consists variables and functions needed for the simulation of spacecraft attitude dynamics

# Usage
```
# Include module `AttitudeDynamics`
include("AttitudeDynamics.jl")
using .AttitudeDynamics

...

# Use struct inside module
AttitudeDynamics.DynamicsModel(Inertia, Torque)
```
"""
module AttitudeDynamics


"""
    DynamicsModel(InertiaMatrix::Matrix, DisturbanceTorque::Matrix)

mutable struct of attitude dynamics model
- InertiaMatrix: inertia matrix of a given system
- DisturbanceTorque: disturbance torque to the system
"""
mutable struct DynamicsModel
    # 慣性ダイアディック
    InertiaMatrix::Matrix

    # 外乱トルク
    DisturbanceTorque::Vector
end

# Equation of dynamics
"""
    diffDynamics(model::DynamicsModel, currentTime, currentOmega, currentCoordB)

Get the differential of equation of dynamics.

# Arguments
- model::DynamicsModel
- currentTime: current time of system [s]
- currentOmega: angular velocity of system [rad/s]
- currentCoordB: current coordinate matrix [b1 b2 b3]
"""
function diffDynamics(model::DynamicsModel, currentTime, currentOmega, currentCoordB)

    # skew matrix of angular velocity vector
    skewOmega = [
        0 -currentOmega[3] currentOmega[2]
        currentOmega[3] 0 -currentOmega[1]
        -currentOmega[2] currentOmega[1] 0]

    differential = inv(model.InertiaMatrix) * (model.DisturbanceTorque - currentCoordB' * model.InertiaMatrix * skewOmega * currentCoordB * currentCoordB' * currentOmega)

    return differential
end


# Equation of Quaternion
"""
    diffQuaternion(omega::Vector, quaterion::Vector)

Get differential of quaternion from equation of kinematics

# Arguments
- omega: angular velocity of system
- quaterion: current value of quaternion
"""
function diffQuaternion(omega, quaternion)

    OMEGA = [
        0 omega[3] -omega[2] omega[1]
        -omega[3] 0 omega[1] omega[2]
        omega[2] -omega[1] 0 omega[3]
        -omega[1] -omega[2] -omega[3] 0
    ]

    differential = 1/2 * OMEGA * quaternion

    return differential
end

# Update the angular velocity (time evolution)
"""
    updateAngularVelocity(model::DynamicsModel, currentTime, currentOmega, samplingTime, currentCoordB)

calculate angular velocity at next time step using 4th order Runge-Kutta method
"""
function updateAngularVelocity(model::DynamicsModel, currentTime, currentOmega, samplingTime, currentCoordB)
    # Update the angular velocity vector using 4th order runge kutta method

    k1 = diffDynamics(model, currentTime                 , currentOmega                      , currentCoordB)
    k2 = diffDynamics(model, currentTime + samplingTime/2, currentOmega + samplingTime/2 * k1, currentCoordB)
    k3 = diffDynamics(model, currentTime + samplingTime/2, currentOmega + samplingTime/2 * k2, currentCoordB)
    k4 = diffDynamics(model, currentTime + samplingTime  , currentOmega + samplingTime   * k3, currentCoordB)

    nextOmega = currentOmega + samplingTime/6 * (k1 + 2*k2 + 2*k3 + k4)

    return nextOmega
end


# Update the quaternion vector (time evolution)
"""
    updateQuaternion(currentOmega, currentQuaternion, samplingTime)

calculate quaternion at next time step using 4th order Runge-Kutta method.
"""
function updateQuaternion(currentOmega, currentQuaternion, samplingTime)
    # Update the quaterion vector using 4th order runge kutta method

    k1 = diffQuaternion(currentOmega, currentQuaternion                      );
    k2 = diffQuaternion(currentOmega, currentQuaternion + samplingTime/2 * k1);
    k3 = diffQuaternion(currentOmega, currentQuaternion + samplingTime/2 * k2);
    k4 = diffQuaternion(currentOmega, currentQuaternion + samplingTime   * k3);

    nextQuaternion = currentQuaternion + samplingTime/6 * (k1 + 2*k2 + 2*k3 + k4);

    return nextQuaternion
end

"""
    getTransformationMatrix(q)

Calculate the transformation matrix from coordinate A system (inertial frame) to coordinate B system (spacecraft body fixed frame).

# Arguments
- `q`: quaternion
"""
function getTransformationMatrix(q)

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

    C = [
        q[1]^2 - q[2]^2 - q[3]^2 + q[4]^2  2*(q[1]*q[2] + q[3]*q[4])          2*(q[1]*q[3] - q[2]*q[4])
        2*(q[2]*q[1] - q[3]*q[4])          q[2]^2 - q[3]^2 - q[1]^2 + q[4]^2  2*(q[2]*q[3] + q[1]*q[4])
        2*(q[3]*q[1] + q[2]*q[4])          2*(q[3]*q[2] - q[1]*q[4])          q[3]^2 - q[1]^2 - q[2]^2 + q[4]^2
    ]

    return C
end


end
