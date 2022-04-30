"""
    module Disturbance

Module that includes functions related to the calculation of disturbance to the system
"""
module Disturbance

using LinearAlgebra:norm
using StaticArrays

export DisturbanceConfig, disturbanceinput

"""
    struct DisturbanceConfig

Struct that configures disturbance torque to the spacecraft
"""
struct DisturbanceConfig

    consttorque::Union{SVector{3, <:Real}, Nothing}
    gravitationaltorque::Bool

    # Constructor
    DisturbanceConfig(; constanttorque = zeros(3), gravitygradient = false) = begin

        # Create constant torque with respect to the body frame
        constanttorque = SVector{3, Float64}(constanttorque)

        return new(constanttorque, gravitygradient)
    end

end

"""
    constant_torque(constant_torque::Vector)

Function that returns given constant torque vector
"""
constant_torque(C_ECI2Body::Union{Matrix{<:Real}, SMatrix{3, 3, <:Real}}, constant_torque::SVector{3, <:Real})::SVector{3} = transpose(C_ECI2Body) * constant_torque

"""
    constant_torque(C_ECI2Body::SMatrix{3, 3, <:Real}, constant_torque::Nothing)

Function that returns zero vector
"""
constant_torque(C_ECI2Body::Union{Matrix{<:Real}, SMatrix{3, 3, <:Real}}, constant_torque::Nothing)::SVector{3} = zeros(SVector{3})

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
function disturbanceinput(distconfig::DisturbanceConfig, inertia, orbit_angular_velocity, C_ECI2Body, C_ECI2LVLH, LVLHframe_z)::Vector

    disturbance = zeros(3) + constant_torque(C_ECI2Body, distconfig.consttorque)

    if distconfig.gravitationaltorque == true
        disturbance = disturbance + gravity_gradient_torque(inertia, orbit_angular_velocity, C_ECI2Body, C_ECI2LVLH, LVLHframe_z)
    end

    return disturbance
end

end
