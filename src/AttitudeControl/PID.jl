"""
    PID

submodule that contains implementation of the Proportional-Integral-Differential (PID) controller
"""
module PID

struct PIDconfig
    # gains
    Pgain::AbstractVecOrMat
    Igain::AbstractVecOrMat
    Dgain::AbstractVecOrMat
end

function define_controller(config::AbstractDict)
    # check the configuration
    if haskey(config, "Pgain") thorw(KeyError("key \"Pgain\" is not configured")) end
    if haskey(config, "Igain") thorw(KeyError("key \"Igain\" is not configured")) end
    if haskey(config, "Dgain") thorw(KeyError("key \"Dgain\" is not configured")) end

    controller_config = PIDconfig(
        config["Pgain"],
        config["Igain"],
        config["Dgain"]
    )

    return controller_config
end

function get_controlinput(config::PIDconfig, state::Union{AbstractVector, Real}, target::Union{AbstractVector, Real})

    error = calc_error(state, target)
    integraldata = 0;

    input = config.Pgain * error + config.Igain * integraldata + config.Dgain * error

    return input
end

function calc_error(state::Union{AbstractVector, Real}, target::Union{AbstractVector, Real})

    error = state - target

    return error
end

end
