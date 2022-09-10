module AttitudeControlBase

using Reexport

include("PID.jl")
@reexport using .PID

export set_attitudecontroller

function _set_controller(strategy::AbstractString, config::AbstractDict)

    if strategy == "PID"
        attitude_controller = PID.set_controller(config)
    else
        throw(ArgumentError("control strategy \"$strategy\" is invalid"))
    end

    return attitude_controller
end

function _init_controller_internals(strategy::AbstractString, config::AbstractDict)

    if strategy == "PID"
        internals = PID.init_internals(config)
    else
        throw(ArgumentError("control strategy \"$strategy\" is invalid"))
    end

    return internals
end

function set_attitudecontroller(paramdict::AbstractDict)

    # set controller configuration
    attitude_controller = _set_controller(paramdict["strategy"], paramdict["config"])

    # initialize controller internals
    attitude_controller_internals = _init_controller_internals(paramdict["strategy"], paramdict["config"])

    return (attitude_controller, attitude_controller_internals)
end

end
