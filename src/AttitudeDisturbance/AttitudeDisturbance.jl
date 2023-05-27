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
include("GravityGradient.jl")

export AttitudeDisturbanceInfo, AttitudeDisturbanceData, calc_attitudedisturbance!, set_attitudedisturbance, init_attitude_disturbance_data


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

    # configuration of the disturbance input
    steptraj::_InternalsStepTrajectory

    # constructor of the `DisturbanceInternals`
    DisturbanceInternals() = begin
        # initialize `steptraj` internal data container
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
    AttitudeDisturbanceData

data container for the attitude disturbance
"""
struct AttitudeDisturbanceData
    all::Vector{SVector{3, <:Real}} # sum of the all disturbance component
    constant::SVector{3, Float64} # constant disturbance
    trajectory::Union{Vector{SVector{3, Float64}}, Nothing} # time-series of the predefined disturbance trajectory
    gravity_gradient::Union{Vector{SVector{3, Float64}}, Nothing} # time-series of the gravity gradient disturbance torque
end

"""
    init_attitude_disturbance_data

initialize data container for the attitude disturbance
"""
function init_attitude_disturbance_data(datanum::Int, info::AttitudeDisturbanceInfo)

    # copy the constant torque info
    constant = info.config.consttorque

    # initialize time-series trajectory
    trajectory = [SVector{3}(zeros(3)) for _ in 1:datanum]

    # gravity gradient torque
    if info.config.gravitationaltorque
        gravity = [SVector{3}(zeros(3)) for _ in 1:datanum]
    else
        gravity = nothing
    end

    # disturbance applied to the spacecraft
    all = [SVector{3}(zeros(3)) for _ in 1:datanum]

    return AttitudeDisturbanceData(all, constant, trajectory, gravity)

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

calculate disturbance torque for the attitude dynamics
"""
function calc_attitudedisturbance!(
    info::AttitudeDisturbanceInfo,
    attitudemodel,
    attidistdata::AttitudeDisturbanceData,
    simcnt::Unsigned,
    currenttime::Real,
    C_ECI2Body::SMatrix{3, 3, <:Real},
    C_ECI2LVLH::SMatrix{3, 3, <:Real},
    altitude::Real,
    Tsampling::Real
    )::SVector{3, Float64}

    # earth direction in ECI frame
    nadir_earth = C_ECI2Body * transpose(C_ECI2LVLH) * SVector{3}([0.0, 0.0, -1.0])

    # apply step trajectory torque
    if !isnothing(info.config.steptraj)
        trajectory = _step_trajectory!(info.config.steptraj, info.internals.steptraj, currenttime, Tsampling)
        attidistdata.trajectory[simcnt] = trajectory
    else
        trajectory = SVector{3}(zeros(3))
    end

    # apply gravitational torque
    if info.config.gravitationaltorque == true
        gravity_torque = _gravity_gradient(attitudemodel, nadir_earth, altitude)
        attidistdata.gravity_gradient[simcnt] = gravity_torque
    else
        gravity_torque = SVector{3}(zeros(3))
    end

    # sum up all disturbance component
    disturbance_torque = _constant_torque(info.config.consttorque) + trajectory + gravity_torque
    attidistdata.all[simcnt] = disturbance_torque

    return disturbance_torque
end

end
