module ParameterSettingBase

using YAML
using ..Frames, ..OrbitBase, ..DynamicsBase, ..KinematicsBase, ..AttitudeDisturbance, ..StructuresBase, ..AttitudeControlBase

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
    readparamfile

read the configuration of the spacecraft from the configuration file in YAML format.

# Arguments

* `filepath::String`: Path to the configuration file

# Usage

```julia
paramfilepath = "./test/spacecraft2.yml"
(simconfig, attitudemodel, distconfig, initvalue, orbitinfo, strparam, strmodel) = readparamfile(paramfilepath)
```
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
        (distconfig, distinternals) = set_attitudedisturbance(paramread["disturbance"])
    else
        throw(AssertionError("disturbance configuration is not found on parameter setting file"))
    end

    # Orbital dynamics
    if haskey(paramread, "Orbit")
        orbitinfo = setorbit(paramread["Orbit"], ECI_frame)
    else
        throw(AssertionError("orbit configuration is not found on parameter setting file"))
    end

    # Flexible appendage
    if haskey(paramread, "appendage")
        (strparam, strmodel, strdistconfig, strinternals) = setstructure(paramread["appendage"])
    end

    # Attitude controller
    if haskey(paramread, "attitude controller")
        attitude_controller = set_attitudecontroller(paramread["attitude controller"])
    else
        throw(AssertionError("attitude controller configuration is not found on parameter setting file"))
    end

    return (simconfig, attimodel, distconfig, distinternals, initvalue, orbitinfo, strparam, strmodel, strdistconfig, strinternals, attitude_controller)
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
