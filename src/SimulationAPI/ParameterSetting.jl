module ParameterSetting

using YAML

using ..Frames
using ..Orbit
using ..RigidBody, ..LinearCoupling
using ..Attitude
using ..Disturbance

export SimulationConfig, setorbit, setdynamicsmodel, setsimconfig, setinitvalue, setdisturbance


"""
    struct SimulationConfig

struct that contains the information about the simulation configuration

## fields
* `simulationtime::Real`: time length of the simulation
* `samplingtime::Real`: sampling time of the simulation
"""
struct SimulationConfig
    simulationtime::Real
    samplingtime::Real
end

"""
    setorbit(filepath::String, ECI::Frame)::OrbitInfo

Load the YAML file configuration and construct the appropriate model for the simulation
"""
function setorbit(filepath::String, ECI::Frame)::OrbitInfo

    # Read YAML file
    lawread = YAML.load_file(filepath)

    if haskey(lawread, "property") == false
        throw(ErrorException("`property` is not specified in YAML configuration file"))
    elseif lawread["property"] != "orbit"
        throw(AssertionError("`property` does not match with `orbit`"))
    end

    # Define Orbit.OrbitInfo
    orbitinfo = OrbitInfo(
        OrbitalElements(
            lawread["OrbitalElements"]["right ascention"],
            lawread["OrbitalElements"]["inclination"],
            lawread["OrbitalElements"]["semimajor axis"],
            lawread["OrbitalElements"]["eccentricity"],
            lawread["OrbitalElements"]["argument of perigee"],
            lawread["OrbitalElements"]["true anomaly at epoch"]
        ),
        ECI,
        lawread["OrbitInfo"]
    )

    return orbitinfo
end

"""
    setdynamicsmodel(filepath::String)

Load the YAML file configuration and construct the appropriate model for the simulation
"""
function setdynamicsmodel(filepath::String)

    lawdata = YAML.load_file(filepath)

    if haskey(lawdata, "property") == false
        throw(ErrorException("`property` is not specified in YAML configuration file"))
    elseif lawdata["property"] != "dynamics"
        throw(AssertionError("`property` does not match with `dynamics`"))
    end

    if lawdata["dynamicsmodel"] == "Linear coupling"
        inertia = reshape(lawdata["platform"]["inertia"], (3,3))

        # get dimension of the structural motion of the flexible appendages
        dimstructurestate = Int(length(lawdata["platform"]["coupling"]) / 3)

        Dcplg = reshape(lawdata["platform"]["coupling"], (3, dimstructurestate))

        model = LinearCouplingModel(inertia, Dcplg, dimstructurestate)
    else
        error("configuration for dynamics model in YAML file is set improperly")
    end

    return model
end

"""
    setsimconfig(filepath::String)::SimulationConfig

initialize the simulation configurations

## Return value

* `simconfig::SimulationConfig`
"""
function setsimconfig(filepath::String)::SimulationConfig

    # Read configuration file
    lawread = YAML.load_file(filepath)

    if haskey(lawread, "property") == false
        throw(ErrorException("`property` is not specified in YAML configuration file"))
    elseif lawread["property"] != "simconfig"
        throw(AssertionError("`property` does not match with `simconfig`"))
    end

    # set values
    samplingtime = lawread["sampling time"]
    simulationtime = lawread["simulation time"]

    # struct for configurations of simulation
    simconfig = SimulationConfig(simulationtime, samplingtime)

    return simconfig
end

"""
    setinitvalue(filepath::String)::InitData

Define the inital value for simulation
"""
function setinitvalue(filepath::String)::InitData

    lawread = YAML.load_file(filepath)

    if haskey(lawread, "property") == false
        throw(ErrorException("`property` is not specified in YAML configuration file"))
    elseif lawread["property"] != "initvalue"
        throw(AssertionError("`propety` does not match with `initvalue`"))
    end

    initvalue = InitData(
        lawread["quaternion"],
        lawread["angular velocity"],
        ECI_frame
    )

    return initvalue
end

"""
    setdisturbance(filepath::String)::DisturbanceConfig

set disturbance configuration from YAML setting file
"""
function setdisturbance(filepath::String)::DisturbanceConfig

    lawread = YAML.load_file(filepath)

    if haskey(lawread, "property") == false
        throw(ErrorException("`property` is not specified in YAML configuration file"))
    elseif lawread["property"] != "distconfig"
        throw(AssertionError("`property` deos not match with `distconfig`"))
    end

    distconfig = DisturbanceConfig(
        constanttorque = lawread["constant torque"],
        gravitygradient = lawread["gravitational torque"]
    )

    return distconfig
end

end
