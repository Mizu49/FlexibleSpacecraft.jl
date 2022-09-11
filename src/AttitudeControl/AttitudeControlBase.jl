module AttitudeControlBase

using Reexport

include("PID.jl")
@reexport using .PID

export set_attitudecontroller

function set_attitudecontroller(paramdict::AbstractDict)

    control_strategy = paramdict["strategy"]

    # set controller
    if control_strategy == "PID"
        attitude_controller = PIDController(paramdict["config"])
    else
        error("no matching attitude control strategy for $control_strategy")
    end

    return attitude_controller
end

end
