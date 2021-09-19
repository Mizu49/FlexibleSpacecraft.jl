"""
    module Disturbance

Module that includes functions related to the calculation of disturbance to the system
"""
module Disturbance

using LinearAlgebra:norm

export DisturbanceConfig, disturbanceinput

"""
    struct DisturbanceConfig

Struct that configures disturbance torque to the spacecraft
"""
struct DisturbanceConfig

    consttorque::Union{Vector, Nothing}
    gravitationaltorque::Bool

    # Constructor
    DisturbanceConfig() = begin
        consttorque = nothing
        gravitationaltorque = false

        return new(consttorque, gravitationaltorque)
    end

end

"""
    constant_torque(constant_torque::Vector)

Function that returns given constant torque vector
"""
constant_torque(constant_torque::Vector)::Vector = constant_torque

"""
    constant_torque(constant_torque::Nothing)

Function that returns zero vector
"""
constant_torque(constant_torque::Nothing)::Vector = zeros(3)

"""
    function gravity_gradient_torque(inertia, orbit_angular_velocity, C_ECI2Body, C_ECI2LVLH, LVLHframe_z)

Function that returns gravity gradient torque

# Arguments

* Inertia matrix
* Angular velocity of the orbit
* Transfromation matrix from ECI frame to body frame
* Transformation matrix from ECI frame to LVLH frame
* Z-vector of LVLH frame

"""
function gravity_gradient_torque(inertia, orbit_angular_velocity, C_ECI2Body, C_ECI2LVLH, LVLHframe_z)

    # Transformation matrix from LVLH to spacecraft body frame
    C = C_ECI2Body * inv(C_ECI2LVLH)

    # calculate nadir vector
    nadir_vector = C * LVLHframe_z

    # make it unit vector
    nadir_vector = 1/norm(nadir_vector) .* nadir_vector

    nadir_skew = [
         0 -nadir_vector[3]  nadir_vector[2]
         nadir_vector[3] 0  -nadir_vector[1]
        -nadir_vector[2]  nadir_vector[1] 0
    ]

    torque_vector = 3*orbit_angular_velocity^2 * nadir_skew * inertia * nadir_vector;

    return torque_vector
end

"""

"""
function disturbanceinput(distconfig::DisturbanceConfig)::Vector

    disturbance = zeros(3) + constant_torque(distconfig.consttorque)

    return disturbance
end

end
