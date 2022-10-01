"""
    NoAttitudeControl

submodule that contains implementation of the no attitude controller
"""
module NoAttitudeControl

using ..Utilities

export NoAttitudeController

mutable struct NoAttitudeController
    config::Any
    NoAttitudeController() = begin

        controller_config = nothing

        new(controller_config)
    end
end

@inline function control_input!(controller::NoAttitudeController, state::Union{AbstractVector, Real}, target::Union{AbstractVector, Real})

    return [0, 0, 0]
end

end
