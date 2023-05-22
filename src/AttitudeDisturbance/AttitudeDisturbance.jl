"""
    AttitudeDisturbance

Submodule that provides the features for calculating the disturbance torque to the attitude dynamics
"""
module AttitudeDisturbance

using LinearAlgebra:norm
using StaticArrays
using ..Frames

# include supporting program files
include("DisturbanceTrajectory.jl")
include("GravityDisturbance.jl")

export AttitudeDisturbanceInfo, calc_attitudedisturbance, set_attitudedisturbance


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
    attitudemodel,
    currenttime::Real,
    C_ECI2Body::SMatrix{3, 3, <:Real},
    C_ECI2LVLH::SMatrix{3, 3, <:Real},
    altitude::Real,
    Tsampling::Real
    )::SVector{3, Float64}

    # earth direction in ECI frame
    nadir_earth = C_ECI2Body * transpose(C_ECI2LVLH) * SVector{3}([0.0, 0.0, -1.0])

    # initialize disturbance torque vector
    disturbance = SVector{3, Float64}(zeros(3))

    # apply constant torque
    disturbance = disturbance + _constant_torque(info.config.consttorque)

    # apply step trajectory torque
    if !isnothing(info.config.steptraj)
        disturbance = disturbance + _step_trajectory!(info.config.steptraj, info.internals.steptraj, currenttime, Tsampling)
    end

    # apply gravitational torque
    if info.config.gravitationaltorque == true
        disturbance = disturbance + _gravity_gradient(attitudemodel, C_ECI2Body, nadir_earth, altitude)
    end

    return disturbance
end

end
