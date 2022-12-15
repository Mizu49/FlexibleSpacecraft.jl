module DynamicsBase

using Reexport, StaticArrays
using ..Frames, ..UtilitiesBase

export AbstractAttitudeDynamicsModel, update_angularvelocity, setdynamicsmodel, calc_angular_momentum

include("RigidBody.jl")
@reexport using .RigidBody

include("LinearCoupling.jl")
@reexport using .LinearCoupling

AbstractAttitudeDynamicsModel = Union{RigidBodyModel, LinearCouplingModel}

"""
    setdynamicsmodel

Load a dictionaly data of configuration and construct the appropriate model for the simulation for the attitude dynamics
"""
function setdynamicsmodel(paramsetting::AbstractDict)

    if paramsetting["model"] == "Linear coupling"

        inertia = yamlread2matrix(paramsetting["inertia"], (3,3))

        # get dimension of the structural motion of the flexible appendages
        dimstructurestate = Int(length(paramsetting["coupling"]) / 3)
        Dcplg = yamlread2matrix(paramsetting["coupling"], (3, dimstructurestate))

        model = LinearCouplingModel(inertia, Dcplg, dimstructurestate)

    elseif paramsetting["model"] == "Rigid body"

        inertia = yamlread2matrix(paramsetting["inertia"], (3,3))

        model = RigidBodyModel(inertia)
    else
        error("configuration for dynamics model in YAML file is set improperly")
    end

    return model
end


"""
    update_angularvelocity

update the angular velocity of the angular velocity of the attitude dynamics. Interface to the individual functions implemented in each submodules

# Arguments

* `model`: dynamics model for the atittude dynamics
* `currentTime::Real`: current time
* `angularvelocity::AbstractVector{<:Real}`: angular velocity vector
* `Tsampling::Real`: sampling period for the dynamics simulation
* `currentbodyframe::Frame`: current frame variable of the spacecraft's body fixed frame
* `distinput::AbstractVector{<:Real}`: disturbance input vector
* `straccel::AbstractVector{<:Real}`: acceleration of the structural response of the flexible appendages
* `strvelocity::AbstractVector{<:Real}`: velocity of the structural response of the flexible appendages

"""
function update_angularvelocity(
    model::AbstractAttitudeDynamicsModel,
    currentTime::Real,
    angularvelocity::AbstractVector{<:Real},
    Tsampling::Real,
    distinput::AbstractVector{<:Real},
    ctrlinput::AbstractVector{<:Real},
    straccel::Union{AbstractVector{<:Real}, Real},
    strvelocity::Union{AbstractVector{<:Real}, Real}
    )::SVector{3, <:Real}

    # check size of vectors
    check_size(angularvelocity, 3)
    check_size(distinput, 3)
    check_size(ctrlinput, 3)

    # switch based on the type of `model`
    if typeof(model) == RigidBodyModel
        angularvelocity = RigidBody.update_angularvelocity(model, currentTime, angularvelocity, Tsampling, distinput, ctrlinput)
    elseif typeof(model) == LinearCouplingModel
        angularvelocity = LinearCoupling.update_angularvelocity(model, currentTime, angularvelocity, Tsampling, distinput, ctrlinput, straccel, strvelocity)
    else
        error("given model is invalid")
    end

    return angularvelocity
end

function calc_angular_momentum(
    model::AbstractAttitudeDynamicsModel,
    angular_velocity::AbstractVector{<:Real}
    )::SVector{3, <:Real}

    # check size of the vector
    check_size(angular_velocity, 3)

    # switch based on the type of model
    if typeof(model) == RigidBodyModel
        L = RigidBody.calc_angular_momentum(model, angular_velocity)
    elseif typeof(model) == LinearCouplingModel
        L = LinearCoupling.calc_angular_momentum(model, angular_velocity)
    else
        error("given model is invalid")
    end

    return L
end

end
