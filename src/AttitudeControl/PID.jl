"""
    PID

submodule that contains implementation of the Proportional-Integral-Differential (PID) controller
"""
module PID

using StaticArrays
using ..UtilitiesBase

export PIDController

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
        load_matrix(config["Pgain"]),
        load_matrix(config["Igain"]),
        load_matrix(config["Dgain"])
    )

    return controller_config
end

function _init_controller_internals(config::AbstractDict)
    return _Internals(zeros(3), zeros(3))
end

@inline function control_input!(controller::PIDController, currentRPY::SVector{3, <:AbstractFloat}, targetRPY::SVector{3, <:AbstractFloat})::SVector{3, Float64}

    error = currentRPY - targetRPY
    error_diff = error - controller._internals.previouserror

    # process internal calculation
    controller._internals.previouserror = error
    controller._internals.cumulativeerror = controller._internals.cumulativeerror + error

    input =
        - controller._config.Pgain * error +
        - controller._config.Igain * controller._internals.cumulativeerror +
        - controller._config.Dgain * error_diff

    return input
end

end
