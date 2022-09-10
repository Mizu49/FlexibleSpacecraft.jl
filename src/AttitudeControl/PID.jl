"""
    PID

submodule that contains implementation of the Proportional-Integral-Differential (PID) controller
"""
module PID

export control_input!

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

function set_controller(config::AbstractDict)
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

function init_internals(config::AbstractDict)
    return _Internals(0, 0)
end

@inline function control_input!(config::_Config, internals::_Internals, state::Union{AbstractVector, Real}, target::Union{AbstractVector, Real})

    error = _calc_error(state, target)
    error_diff = (error - internals.previouserror)/0.1

    # process internal calculation
    internals.previouserror = error
    internals.cumulativeerror = internals.cumulativeerror + error

    input =
        - config.Pgain * error +
        - config.Igain * internals.cumulativeerror +
        - config.Dgain * error_diff

    return input
end

@inline function _calc_error(state::Union{AbstractVector, Real}, target::Union{AbstractVector, Real})

    error = state - target

    return error
end

end
