module ParameterSettingBase

using YAML
using ..Frames, ..Orbit, ..DynamicsBase, ..Attitude, ..Disturbance

export SimulationConfig, yamlread2matrix, readparamfile

"""
    struct SimulationConfig

struct that contains the information about the simulation configuration

# fields

* `spacecraft::String`: name of the spacecraft
* `note::String`: note on the parameter settings
* `simulationtime::Real`: time length of the simulation
* `samplingtime::Real`: sampling time of the simulation
"""
struct SimulationConfig
    spacecraft::String
    note::String
    simulationtime::Real
    samplingtime::Real
end

"""
    yamlread2matrix

function that converts the direct read data from YAML file into the Matrix

# Argument

* `x::AbstractVector`: direct read data from YAML
* `size::Tuple{<:Int, <:Int}`: size of the matrix to be converted
"""
@inline function yamlread2matrix(x::AbstractVector, size::Tuple{<:Int, <:Int})
    return Matrix(reshape(x, size)')
end

"""
    readparamfile
"""
function readparamfile(filepath::String)

    # Read YAML file
    paramread = YAML.load_file(filepath)

    # simulation configuration
    if haskey(paramread, "config")
        simconfig = _setsimconfig(paramread["config"])
    else
        throw(AssertionError("simulation configuration is not found on parameter setting file"))
    end

    # initial value
    if haskey(paramread, "initial value")
        initvalue = _setinitvalue(paramread["initial value"])
    else
        throw(AssertionError("initial value configuration is not found on parameter setting file"))
    end

    # Dynamics model
    if haskey(paramread, "attitude dynamics")
        attimodel = setdynamicsmodel(paramread["attitude dynamics"])
    else
        throw(AssertionError("attitude dynamics configuration is not found on parameter setting file"))
    end

    # Disturbance for the attitude dynamics
    if haskey(paramread, "disturbance")
        distconfig = setdisturbance(paramread["disturbance"])
    else
        throw(AssertionError("disturbance configuration is not found on parameter setting file"))
    end

    # Orbital dynamics
    if haskey(paramread, "Orbit")
        orbitinfo = setorbit(paramread["Orbit"], ECI_frame)
    else
        throw(AssertionError("orbit configuration is not found on parameter setting file"))
    end

    return (simconfig, attimodel, distconfig, initvalue, orbitinfo)
end

"""
    _setsimconfig(filepath::String)::SimulationConfig

initialize the simulation configurations

## Return value

* `simconfig::SimulationConfig`
"""
function _setsimconfig(simconfigdict::AbstractDict)::SimulationConfig

    simconfig = SimulationConfig(
        simconfigdict["name"],
        simconfigdict["note"],
        simconfigdict["time length"],
        simconfigdict["sampling time"]
    )

    return simconfig
end

"""
    _setinitvalue(filepath::String)::InitData

Define the initial value for simulation
"""
function _setinitvalue(initvaluedict::AbstractDict)::InitData

    initvalue = InitData(
        initvaluedict["quaternion"],
        initvaluedict["angular velocity"],
        ECI_frame
    )

    return initvalue
end


end
