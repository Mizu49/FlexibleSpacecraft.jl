module ParameterSettingBase

using YAML
using ..Frames, ..DynamicsBase, ..KinematicsBase, ..AttitudeDisturbance, ..AppendagesBase, ..AttitudeControlBase
import ..OrbitBase

export SimulationConfig, load_matrix, set_simulation_parameters

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
    set_simulation_parameters

read the configuration of the spacecraft from the configuration file in YAML format.

# Arguments

* `filepath::String`: Path to the configuration file

# Usage

```julia
paramfilepath = "./test/spacecraft2.yml"
(simconfig, attitudemodel, distconfig, initvalue, orbitinfo, strparam, strmodel) = set_simulation_parameters(paramfilepath)
```
"""
function set_simulation_parameters(filepath::String)

    # Read YAML file
    paramread = YAML.load_file(filepath)

    # simulation configuration
    if haskey(paramread, "config")
        simconfig = _setsimconfig(paramread["config"])
    else
        throw(AssertionError("simulation configuration is not found on parameter setting file"))
    end

    # Orbital dynamics
    if haskey(paramread, "Orbit")
        orbitinfo = OrbitBase.setorbit(paramread["Orbit"], ECI_frame)
    else
        throw(AssertionError("orbit configuration is not found on parameter setting file"))
    end

    # initial value
    if haskey(paramread, "initial value")
        initvalue = _setkinematics(orbitinfo, paramread["initial value"])
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
        attidistinfo = set_attitudedisturbance(paramread["disturbance"])
    else
        throw(AssertionError("disturbance configuration is not found on parameter setting file"))
    end

    # Flexible appendage
    if haskey(paramread, "appendage")
        appendageinfo = setstructure(paramread["appendage"])
    end

    # Attitude controller
    if haskey(paramread, "attitude controller")
        attitude_controller = set_attitudecontroller(paramread["attitude controller"])
    else
        throw(AssertionError("attitude controller configuration is not found on parameter setting file"))
    end

    return (simconfig, attimodel, attidistinfo, initvalue, orbitinfo, appendageinfo, attitude_controller)
end

function set_simulation_parameters(spacecraft::AbstractDict)

    # simulation configuration
    if haskey(spacecraft, "config")
        simconfig = _setsimconfig(spacecraft["config"])
    else
        throw(AssertionError("simulation configuration is not found on parameter setting file"))
    end

    # Orbital dynamics
    if haskey(spacecraft, "Orbit")
        orbitinfo = OrbitBase.setorbit(spacecraft["Orbit"], ECI_frame)
    else
        throw(AssertionError("orbit configuration is not found on parameter setting file"))
    end

    # initial value
    if haskey(spacecraft, "initial value")
        initvalue = _setkinematics(orbitinfo, spacecraft["initial value"])
    else
        throw(AssertionError("initial value configuration is not found on parameter setting file"))
    end

    # Dynamics model
    if haskey(spacecraft, "attitude dynamics")
        attimodel = setdynamicsmodel(spacecraft["attitude dynamics"])
    else
        throw(AssertionError("attitude dynamics configuration is not found on parameter setting file"))
    end

    # Disturbance for the attitude dynamics
    if haskey(spacecraft, "disturbance")
        attidistinfo = set_attitudedisturbance(spacecraft["disturbance"])
    else
        throw(AssertionError("disturbance configuration is not found on parameter setting file"))
    end

    # Flexible appendage
    if haskey(spacecraft, "appendage")
        appendageinfo = setstructure(spacecraft["appendage"])
    end

    # Attitude controller
    if haskey(spacecraft, "attitude controller")
        attitude_controller = set_attitudecontroller(spacecraft["attitude controller"])
    else
        throw(AssertionError("attitude controller configuration is not found on parameter setting file"))
    end

    return (simconfig, attimodel, attidistinfo, initvalue, orbitinfo, appendageinfo, attitude_controller)
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
    _setkinematics(filepath::String)::InitKinematicsData

Define the initial value for simulation
"""
function _setkinematics(orbitinfo::OrbitBase.OrbitInfo, initvaluedict::AbstractDict)::InitKinematicsData

    # calculate the inital quaternion value based on the orbital reference frame
    initquaternion = OrbitBase.calc_inital_quaternion(orbitinfo.orbitalelement, initvaluedict["roll-pitch-yaw"])

    initvalue = InitKinematicsData(
        initquaternion,
        initvaluedict["angular velocity"],
        ECI_frame
    )

    return initvalue
end

function _setkinematics(orbitinfo::Nothing, initvaluedict::AbstractDict)::InitKinematicsData

    # initialize quaternion
    initquaternion = [0.0, 0.0, 0.0, 1.0]

    initvalue = InitKinematicsData(
        initquaternion,
        initvaluedict["angular velocity"],
        ECI_frame
    )

    return initvalue
end

end
