module AttitudeControlBase

using Reexport, StaticArrays
using ..UtilitiesBase

include("NoAttitudeControl.jl")
@reexport using .NoAttitudeControl
include("PID.jl")
@reexport using .PID
include("ConstantInput.jl")
@reexport using .ConstantInput

export AbstractAttitudeController, set_attitudecontroller, control_input!

AbstractAttitudeController = Union{NoAttitudeController, PIDController, ConstantInputController}

function set_attitudecontroller(paramdict::AbstractDict)::AbstractAttitudeController

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

"""
    contorl_input!

function that calculates control input torque to the attitude dynamics

# Arguments

* `controller::AbstractAttitudeController`: instance of attitude controller
* `RPYangle::SVector{3, <:AbstractFloat}`: roll-pitch-yaw angle of the spacecraft attitude with respect to the orbital reference frame
* `targetRPYangle::SVector{3, <:AbstractFloat}`: target roll-pitch-yaw angle of the spacecraft attitude with respect to the orbital reference frame
"""
@inline function control_input!(controller::AbstractAttitudeController, RPYangle::SVector{3, <:AbstractFloat}, targetRPYangle::SVector{3, <:AbstractFloat})::SVector{3, Float64}

    strategy = typeof(controller)

    # switch based on the control strategy
    if strategy == NoAttitudeController
        input = NoAttitudeControl.control_input!(controller, RPYangle, targetRPYangle)
    elseif strategy == PIDController
        input = PID.control_input!(controller, RPYangle, targetRPYangle)
    elseif strategy == ConstantInputController
        input = ConstantInput.control_input!(controller, RPYangle, targetRPYangle)
    else
        error("no matching attitude control strategy for $strategy")
    end

    return input
end

end
