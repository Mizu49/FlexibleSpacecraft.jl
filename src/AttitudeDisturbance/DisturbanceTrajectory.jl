# Attitude disturbance calculation features for predefined disturbance trajectory

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
    _constant_torque(torque_config::AbstractVector{<:Real})

Function that returns the predefined constant disturbance torque vector
"""
_constant_torque(torque_config::AbstractVector{<:Real})::SVector{3} = torque_config

"""
    _constant_torque(torque_config::Nothing)

    Function that returns zero disturbance torque for attitude dynamics
"""
_constant_torque(torque_config::Nothing)::SVector{3} = zeros(SVector{3})


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
