"""
    AttitudeDisturbance

Submodule that provides the features for calculating the disturbance torque to the attitude dynamics
"""
module AttitudeDisturbance

using LinearAlgebra:norm
using StaticArrays
using ..Frames

export AttitudeDisturbanceInfo, calc_attitudedisturbance, set_attitudedisturbance

"""
    _ConfigStepTrajectory

struct that configures the step trajectory of the disturbance torque input
"""
struct _ConfigStepTrajectory
    confignum::Unsigned
    stepvalue::AbstractVector{<:AbstractVector}
    endurance::AbstractVector{<:Real}

    # Constructor
    _ConfigStepTrajectory(stepvalue::AbstractVector, endurance::AbstractVector) = begin
        # check the argument
        if size(stepvalue, 1) != size(endurance, 1)
            throw(ArgumentError("argument vector `stepvalue` and `endurance` have inconsisitent size"))
        end
        # set configuration
        confignum = size(stepvalue, 1)

        new(confignum, stepvalue, endurance)
    end

    _ConfigStepTrajectory(configdict::AbstractDict) = begin
        # read configuration from dictionary
        stepvalue = configdict["value"]
        endurance = configdict["endurance"]
        confignum = size(stepvalue, 1)

        new(confignum, stepvalue, endurance)
    end
end


"""
    DisturbanceConfig

struct that configures disturbance torque to the spacecraft

# Fields

* `consttorque::Union{SVector{3, <:Real}, Nothing}`: constant torque input
* `gravitationaltorque::Bool`: gravity gradient disturbance torque, configure with the boolean (true OR false)
* `steptraj::Union{Nothing, _ConfigStepTrajectory}`: configuration for the step disturbance trajectory

"""
struct DisturbanceConfig

    consttorque::Union{SVector{3, <:Real}, Nothing}
    gravitationaltorque::Bool
    steptraj::Union{Nothing, _ConfigStepTrajectory}

    # Constructor
    DisturbanceConfig(; constanttorque = zeros(3), gravitygradient = false, step_trajectory = nothing) = begin

        # Create constant torque with respect to the body frame
        constanttorque = SVector{3, Float64}(constanttorque)

        # Configure step trajectory
        if isnothing(step_trajectory)
            step_traj_config = nothing
        else
            step_traj_config = _ConfigStepTrajectory(step_trajectory)
        end

        return new(constanttorque, gravitygradient, step_traj_config)
    end
end

"""
    _InternalsStepTrajectory

internals of the step trajectory

# Fields

* `stepcnt::Unsigned`: counter that specifies the current disturbance profile
* `duration::Real`: duration time of the current disturbance profile

"""
mutable struct _InternalsStepTrajectory
    stepcnt::Unsigned
    duration::Real
end

"""
    DisturbanceInternals

internals of the disturbance torque for the attitude dynamics

# Fields

* `steptraj::_InternalsStepTrajectory`: internals of the step disturbance trajectory
"""
mutable struct DisturbanceInternals
    steptraj::_InternalsStepTrajectory

    DisturbanceInternals() = begin
        # initialize `steptraj`
        steptraj = _InternalsStepTrajectory(1, 0)

        new(steptraj)
    end
end

"""
    AttitudeDisturbanceInfo

struct that contains information of the disturbance input torque of the
"""
struct AttitudeDisturbanceInfo
    config::DisturbanceConfig
    internals::DisturbanceInternals
end

"""
    set_attitudedisturbance

set disturbance configuration from YAML setting file

# YAML configuration format

```yaml
disturbance:
    constant torque: {nothing OR "specify vector [0.0, 0.0, 0.0]"}
    gravitational torque: {false OR true}
    step trajectory:
        {
            nothing
        OR
            value: [[10, 0, 0], [0, 0, 0]]
            endurance: [1, 100]
        }
```
"""
function set_attitudedisturbance(distconfigdict::AbstractDict)

    # constant torque
    if haskey(distconfigdict, "constant torque")
        if distconfigdict["constant torque"] == "nothing"
            constant_torque_config = zeros(3)
        else
            constant_torque_config = distconfigdict["constant torque"]
            if size(constant_torque_config, 1) != 3
                error("size of the constant attitude disturbance torque is invalid. It must be 3")
            end
        end
    else
        constant_torque_config = zeros(3)
    end

    # step trajectory
    if haskey(distconfigdict, "step trajectory")
        if distconfigdict["step trajectory"] == "nothing"
            step_trajectory_config = nothing
        else
            step_trajectory_config = distconfigdict["step trajectory"]
        end
    else
        step_trajectory_config = nothing
    end

    # initialize configuration
    distconfig = DisturbanceConfig(
        constanttorque = distconfigdict["constant torque"],
        gravitygradient = distconfigdict["gravitational torque"],
        step_trajectory = step_trajectory_config
    )
    # initialize internals
    distinternals = DisturbanceInternals()

    # summarize into single struct
    distinfo = AttitudeDisturbanceInfo(distconfig, distinternals)

    return distinfo
end

"""
    calc_attitudedisturbance

calculate disturbance torque input for the attitude dynamics

# Arguments

* `attidistinfo::AttitudeDisturbanceInfo`: disturbance information
* `inertia::AbstractMatrix`: inertia of the spacecraft body
* `currenttime::Real`: current time
* `orbit_angular_velocity::Real`: angular velocity of the orbital motion
* `C_ECI2Body::AbstractMatrix`: rotation matrix from ECI to body
* `C_ECI2LVLH::AbstractMatrix`: rotation matrix from ECI to LVLH
* `LVLHframe_z::AbstractVector`: vector of the LVLH z-axix
* `Tsampling::Real`: sampling time in simulation

"""
function calc_attitudedisturbance(
    info::AttitudeDisturbanceInfo,
    inertia::SMatrix{3, 3, <:Real},
    currenttime::Real,
    orbit_angularvelocity::Real,
    C_ECI2Body::SMatrix{3, 3, <:Real},
    C_ECI2LVLH::SMatrix{3, 3, <:Real},
    LVLHframe::Frame,
    Tsampling::Real
    )::SVector{3, Float64}

    # initialize disturbance torque vector
    disturbance = SVector{3, Float64}(zeros(3))

    # apply constant torque
    disturbance = disturbance + _constant_torque(info.config.consttorque)

    # apply step trajectory torque
    if !isnothing(info.config.steptraj)
        disturbance = disturbance + _step_trajectory!(info.config.steptraj, distinternals.steptraj, currenttime, Tsampling)
    end

    # apply gravitational torque
    if info.config.gravitationaltorque == true
        disturbance = disturbance + _gravity_gradient(inertia, orbit_angularvelocity, C_ECI2Body, C_ECI2LVLH, LVLHframe.z)
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
function _gravity_gradient(inertia::AbstractMatrix, orbit_angular_velocity::Real, C_ECI2Body::AbstractMatrix, C_ECI2LVLH::AbstractMatrix, LVLHframe_z::AbstractVector)

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

function _step_trajectory!(config::_ConfigStepTrajectory, internal::_InternalsStepTrajectory, currenttime::Real, Tsampling::Real)
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

    return SVector{3, Real}(currentinput)
end

end
