module AttitudeControlBase

using Reexport

include("PID.jl")
@reexport using .PID

export define_controller

function define_controller(strategy::AbstractString, config::AbstractDict)

    if strategy == "PID"
        attitude_controller = PID.define_controller(config)
    else
        throw(ArgumentError("control strategy \"$strategy\" is invalid"))
    end

    return attitude_controller
end

end
