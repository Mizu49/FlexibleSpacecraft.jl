"""
    NoAttitudeControl

submodule that contains implementation of the no attitude controller
"""
module NoAttitudeControl

using StaticArrays
using ..UtilitiesBase

export NoAttitudeController

mutable struct NoAttitudeController
    config::Any
    NoAttitudeController() = begin

        controller_config = nothing

        new(controller_config)
    end
end

@inline function control_input!(controller::NoAttitudeController, currentRPY::SVector{3, <:AbstractFloat}, targetRPY::SVector{3, <:AbstractFloat})

    return SVector{3, Float64}([0.0, 0.0, 0.0])
end

end
