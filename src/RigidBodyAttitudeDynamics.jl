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
    calc_differential_dynamics(model::DynamicsModel, currentTime, angular_velocity, current_body_frame)

Get the differential of equation of dynamics.

# Arguments
- model::DynamicsModel
- currentTime: current time of system [s]
- angular_velocity: angular velocity of body frame with respect to ECI frame [rad/s]
- current_body_frame: current body frame [b1 b2 b3]

# return
- differential: differential of equation of motion
"""
function calc_differential_dynamics(model::DynamicsModel, currentTime, angular_velocity, current_body_frame, disturbance)

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
    update_angular_velocity(model::DynamicsModel, currentTime, angular_velocity, Tsampling, current_body_frame)

calculate angular velocity at next time step using 4th order Runge-Kutta method
"""
function calc_angular_velocity(model::DynamicsModel, currentTime, angular_velocity::Vector, Tsampling, current_body_frame::Matrix, disturbance::Vector)
    # Update the angular velocity vector using 4th order runge kutta method

    k1 = calc_differential_dynamics(model, currentTime              , angular_velocity                   , current_body_frame, disturbance)
    k2 = calc_differential_dynamics(model, currentTime + Tsampling/2, angular_velocity + Tsampling/2 * k1, current_body_frame, disturbance)
    k3 = calc_differential_dynamics(model, currentTime + Tsampling/2, angular_velocity + Tsampling/2 * k2, current_body_frame, disturbance)
    k4 = calc_differential_dynamics(model, currentTime + Tsampling  , angular_velocity + Tsampling   * k3, current_body_frame, disturbance)

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

"""
    ECI2BodyFrame(q)

Calculate the transformation matrix from ECI frame to spacecraft body-fixed frame.

# Arguments
- `q`: quaternion

# Return
- `transformation_matrix`: transformation matrix
"""
function ECI2BodyFrame(q)

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


function constant_torque()

    # Calculate constant torque
    torque_vector = [0, 0, 0.1];

    return torque_vector
end

function gravity_gradient_torque(inertia, angular_velocity, nadir, q)

    # Calculate gravity gradient torque

    C = ECI2BodyFrame(q)

    nadir_body = C * nadir

    skewNadir = [
         0 -nadir_body[3]  nadir_body[2]
         nadir_body[3] 0  -nadir_body[1]
        -nadir_body[2]  nadir_body[1] 0
    ]

    torque_vector = 3*angular_velocity^2 * skewNadir * inertia * nadir_body;

    return torque_vector
end

end
