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

end
