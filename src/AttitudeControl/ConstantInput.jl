module ConstantInput

using StaticArrays
export ConstantInputController

struct _Config
    # constant value
    constant_input::Union{AbstractVecOrMat, Real}
end

mutable struct ConstantInputController
    _config::_Config

    ConstantInputController(argconfig::AbstractDict) = begin

        controller_config = _set_controller_config(argconfig)

        new(controller_config)
    end
end

function _set_controller_config(config::AbstractDict)
    # check the configuration
    if !haskey(config, "constant value") throw(KeyError("key \"constant value\" is not configured")) end

    controller_config = _Config(config["constant value"])

    return controller_config
end

@inline function control_input!(controller::ConstantInputController, currentRPY::SVector{3, <:AbstractFloat}, targetRPY::SVector{3, <:AbstractFloat})

    input = controller._config.constant_input

    return input
end

end
