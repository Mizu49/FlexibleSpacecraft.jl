"""
submodule for the disturbance input to the flexible appendages
"""
module StructureDisturbance

abstract type AbstractAppendageDisturbance end

export VibrationConfig, NoStrDisturbance, calcstrdisturbance, setstrdistconfig, AbstractAppendageDisturbance

"""
struct of parameter configuration of the vibration disturbance input
"""
struct VibrationConfig<:AbstractAppendageDisturbance

    # dimension of the disturbance input
    dimdistinput::Int
    # dimension of the disturbance component
    dimdistcomp::Int

    # parameters of the vibration
    frequency::AbstractVector{<:Real}
    amplitude::AbstractVector{<:Real}
    phase::AbstractVector{<:Real}

end

struct NoStrDisturbance<:AbstractAppendageDisturbance
    dimdistinput::Int
end


function calcstrdisturbance(config::AbstractAppendageDisturbance, time::Real)

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

function _calcstrdisturbance(config::NoStrDisturbance, time::Real)

    if config.dimdistinput == 1
        return 0
    else
        return zeros(config.dimdistinput)
    end
end

function setstrdistconfig(configdata::AbstractDict)::AbstractAppendageDisturbance

    if configdata["type"] == "vibration"
        config = _setconfig_vibration(configdata)
    elseif configdata["type"] == "no disturbance"
        config = _setconfig_nodisturbance(configdata)
    else
        error("configuration for disturbance input to structual appendage is invalid")
    end

    return config
end

function _setconfig_vibration(configdata::AbstractDict)

    config = VibrationConfig(
        configdata["dimension"],
        size(configdata["angular velocity"], 1),
        configdata["angular velocity"],
        configdata["amplitude"],
        configdata["phase"]
    )

    return config
end

function _setconfig_nodisturbance(configdata::AbstractDict)

    config = NoStrDisturbance(
        configdata["dimension"]
    )

    return config
end

end
