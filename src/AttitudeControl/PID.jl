"""
    PID

submodule that contains implementation of the Proportional-Integral-Differential (PID) controller
"""
module PID

export PIDController, control_input!

struct _Config
    # gains
    Pgain::Union{AbstractVecOrMat, Real}
    Igain::Union{AbstractVecOrMat, Real}
    Dgain::Union{AbstractVecOrMat, Real}
end

"""
    _Internals

Internal struct for dealing with the controller-related tasks
"""
mutable struct _Internals
    #  Cumulative error for the I controller
    cumulativeerror::Union{AbstractVector, Real}

    #  previous error for the D controller
    previouserror::Union{AbstractVector, Real}
end

mutable struct PIDController
    _config::_Config
    _internals::_Internals

    PIDController(argconfig::AbstractDict) = begin

        controller_config = _set_controller_config(argconfig)
        internals = _init_controller_internals(argconfig)

        new(controller_config, internals)
    end
end

function _set_controller_config(config::AbstractDict)
    # check the configuration
    if !haskey(config, "Pgain") throw(KeyError("key \"Pgain\" is not configured")) end
    if !haskey(config, "Igain") throw(KeyError("key \"Igain\" is not configured")) end
    if !haskey(config, "Dgain") throw(KeyError("key \"Dgain\" is not configured")) end

    controller_config = _Config(
        config["Pgain"],
        config["Igain"],
        config["Dgain"]
    )

    return controller_config
end

function _init_controller_internals(config::AbstractDict)
    return _Internals(0, 0)
end

@inline function control_input!(controller::PIDController, state::Union{AbstractVector, Real}, target::Union{AbstractVector, Real})

    error = _calc_error(state, target)
    error_diff = (error - controller.internals.previouserror)/0.1

    # process internal calculation
    controller.internals.previouserror = error
    controller.internals.cumulativeerror = controller.internals.cumulativeerror + error

    input =
        - controller.config.Pgain * error +
        - controller.config.Igain * internals.cumulativeerror +
        - controller.config.Dgain * error_diff

    return input
end

@inline function _calc_error(state::Union{AbstractVector, Real}, target::Union{AbstractVector, Real})

    error = state - target

    return error
end

end
