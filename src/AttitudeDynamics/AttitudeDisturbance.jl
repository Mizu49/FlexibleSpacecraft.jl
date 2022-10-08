"""
    module AttitudeDisturbance

Module that includes functions related to the calculation of disturbance to the system
"""
module AttitudeDisturbance

using LinearAlgebra:norm
using StaticArrays

export DisturbanceConfig, calc_attitudedisturbance, set_attitudedisturbance

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
    set_attitudedisturbance

set disturbance configuration from YAML setting file
"""
function set_attitudedisturbance(distconfigdict::AbstractDict)::DisturbanceConfig

    distconfig = DisturbanceConfig(
        constanttorque = distconfigdict["constant torque"],
        gravitygradient = distconfigdict["gravitational torque"]
    )

    return distconfig
end

"""
    calc_attitudedisturbance

calculate disturbance torque input to the attitude dynamics
"""
function calc_attitudedisturbance(distconfig::DisturbanceConfig, inertia, orbit_angular_velocity, C_ECI2Body, C_ECI2LVLH, LVLHframe_z)::Vector

    disturbance = zeros(3) + _constant_torque(distconfig.consttorque)

    if distconfig.gravitationaltorque == true
        disturbance = disturbance + _gravity_gradient(inertia, orbit_angular_velocity, C_ECI2Body, C_ECI2LVLH, LVLHframe_z)
    end

    return disturbance
end

"""
    _constant_torque(torque_config::AbstractVector{<:Real})

Function that returns the predefined constant disturbance torque vector
"""
_constant_torque(torque_config::AbstractVector{<:Real})::SVector{3} = torque_config

"""
    _constant_torque(torque_config::Nothing)

    Function that returns zero disturbance torque for attitude dynamics
        """
        _constant_torque(torque_config::Nothing)::SVector{3} = zeros(SVector{3})

"""
    _gravity_gradient(inertia, orbit_angular_velocity, C_ECI2Body, C_ECI2LVLH, LVLHframe_z)

Function that returns gravity gradient torque

# Arguments

* Inertia matrix
* Angular velocity of the orbit
* Transfromation matrix from ECI frame to body frame
* Transformation matrix from ECI frame to LVLH frame
* Z-vector of LVLH frame

"""
function _gravity_gradient(inertia, orbit_angular_velocity, C_ECI2Body, C_ECI2LVLH, LVLHframe_z)

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

end
