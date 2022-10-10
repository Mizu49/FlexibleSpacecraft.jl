"""
    module AttitudeDisturbance

Module that includes functions related to the calculation of disturbance to the system
"""
module AttitudeDisturbance

using LinearAlgebra:norm
using StaticArrays

export DisturbanceConfig, DisturbanceInternals, calc_attitudedisturbance, set_attitudedisturbance

struct StepTrajectoryConfig
    confignum::Unsigned
    stepvalue::AbstractVector{<:AbstractVector}
    endurance::AbstractVector{<:Real}

    # Constructor
    StepTrajectoryConfig(stepvalue, endurance) = begin
        # check the argument
        if size(stepvalue, 1) != size(endurance, 1)
            throw(ArgumentError("argument vector `stepvalue` and `endurance` have inconsisitent size"))
        end
        # set configuration
        confignum = size(stepvalue, 1)

        new(confignum, stepvalue, endurance)
    end
end


"""
struct DisturbanceConfig

    Struct that configures disturbance torque to the spacecraft
"""
struct DisturbanceConfig

    consttorque::Union{SVector{3, <:Real}, Nothing}
    gravitationaltorque::Bool
    steptraj::Union{Nothing, StepTrajectoryConfig}

    # Constructor
    DisturbanceConfig(; constanttorque = zeros(3), gravitygradient = false, step_trajectory = nothing) = begin

        # Create constant torque with respect to the body frame
        constanttorque = SVector{3, Float64}(constanttorque)

        # Configure step trajectory
        if isnothing(step_trajectory)
            step_traj_config = nothing
        else
            step_traj_config = 0 #TODO: implement this interface
        end

        return new(constanttorque, gravitygradient, step_traj_config)
    end
end

mutable struct StepTrajectoryInternals
    stepcnt::Unsigned
    duration::Real
end

mutable struct DisturbanceInternals
    steptraj::StepTrajectoryInternals

    DisturbanceInternals() = begin
        # initialize `steptraj`
        steptraj = StepTrajectoryInternals(0, 0)

        new(steptraj)
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
function calc_attitudedisturbance(
    distconfig::DisturbanceConfig,
    inertia,
    orbit_angular_velocity,
    C_ECI2Body,
    C_ECI2LVLH,
    LVLHframe_z
    )::Vector

    # initialize disturbance torque vector
    disturbance = zeros(3)

    # apply constant torque
    disturbance = disturbance + _constant_torque(distconfig.consttorque)

    # apply step trajectory torque
    if !isnothing(distconfig.steptraj)
        disturbance = disturbance + _step_trajectory!(distconfig.steptraj, distinternal.steptraj, currenttime, Tsampling)
    end

    # apply gravitational torque
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

function _step_trajectory!(config::StepTrajectoryConfig, internal::StepTrajectoryInternals, currenttime::Real, Tsampling::Real)
    # check the duration and predefined endurance
    if internal.duration < config.endurance[internal.stepcnt]
        currentinput = config.stepvalue[internal.stepcnt]
        # increment duration time
        internal.duration = internal.duration + Tsampling
    else
        currentinput = config.stepvalue[internal.stepcnt + 1]
        internal.duration = 0
        # increment `stepcnt` to move on to next sequence
        internal.stepcnt = internal.stepcnt + 1
    end

    return currentinput
end

end
