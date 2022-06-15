"""
submodule for the disturbance input to the flexible appendages
"""
module StructureDisturbance

export VibrationConfig, calcstrdisturbance, setstrdistconfig, AbstractStrDistConfig

"""
struct of parameter configuration of the vibration disturbance input
"""
struct VibrationConfig

    # dimension of the disturbance input
    dimdistinput::Int
    # dimension of the disturbance component
    dimdistcomp::Int

    # parameters of the vibration
    frequency::AbstractVector{<:Real}
    amplitude::AbstractVector{<:Real}
    phase::AbstractVector{<:Real}

end

AbstractStrDistConfig = Union{VibrationConfig}

function calcstrdisturbance(config::AbstractStrDistConfig, time::Real)

    distinput = _calcstrdisturbance(config, time)

    return distinput
end

function _calcstrdisturbance(config::VibrationConfig, time::Real)::Real

    if config.dimdistinput == 1
        distinput = 0
    else
        distinput = zeros(config.dimdistinput)
    end

    for idx = 1:(config.dimdistcomp)
        comp = config.amplitude[idx] * sin(config.frequency[idx] * time + config.phase[idx])
        distinput = distinput + comp
    end

    return distinput
end

function setstrdistconfig(configdata::AbstractDict)

    dimdistcomp = size(configdata["angular velocity"], 1)

    config = VibrationConfig(
        configdata["dimension"],
        dimdistcomp,
        configdata["angular velocity"],
        configdata["amplitude"],
        configdata["phase"]
    )

    return config
end

end
