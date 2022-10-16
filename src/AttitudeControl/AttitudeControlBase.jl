module AttitudeControlBase

using Reexport
using ..UtilitiesBase

include("NoAttitudeControl.jl")
@reexport using .NoAttitudeControl
include("PID.jl")
@reexport using .PID
include("ConstantInput.jl")
@reexport using .ConstantInput

export set_attitudecontroller, control_input!

AttitudeControllers = Union{NoAttitudeController, PIDController, ConstantInputController}

function set_attitudecontroller(paramdict::AbstractDict)

    control_strategy = paramdict["strategy"]

    # set controller
    if control_strategy == "none"
        attitude_controller = NoAttitudeController()
    elseif control_strategy == "PID"
        attitude_controller = PIDController(paramdict["config"])
    elseif control_strategy == "Constant input"
        attitude_controller = ConstantInputController(paramdict["config"])
    else
        error("no matching attitude control strategy for $control_strategy")
    end

    return attitude_controller
end

@inline function control_input!(controller::AttitudeControllers, state::Union{AbstractVector, Real}, target::Union{AbstractVector, Real})

    strategy = typeof(controller)
    if strategy == NoAttitudeController
        input = NoAttitudeControl.control_input!(controller, state, target)
    elseif strategy == PIDController
        input = PID.control_input!(controller, state, target)
    elseif strategy == ConstantInputController
        input = ConstantInput.control_input!(controller, state, target)
    else
        error("no matching attitude control strategy for $strategy")
    end

    return input
end

end
