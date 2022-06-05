module ParameterSetting

using YAML

using ..Frames
using ..Orbit
using ..RigidBody, ..LinearCoupling
using ..Attitude
using ..Disturbance

export SimulationConfig, setorbit, setdynamicsmodel, setsimconfig, setinitvalue, setdisturbance


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
