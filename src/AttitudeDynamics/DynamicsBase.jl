module DynamicsBase

using Reexport, StaticArrays
using ..Frames, ..Utilities

export update_angularvelocity, setdynamicsmodel

include("RigidBody.jl")
@reexport using .RigidBody

include("LinearCoupling.jl")
@reexport using .LinearCoupling

TypeModels = Union{RigidBodyModel, LinearCouplingModel}

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
    model::TypeModels,
    currentTime::Real,
    angularvelocity::AbstractVector{<:Real},
    Tsampling::Real,
    currentbodyframe::Frame,
    distinput::AbstractVector{<:Real},
    ctrlinput::AbstractVector{<:Real},
    straccel::AbstractVector{<:Real},
    strvelocity::AbstractVector{<:Real}
    )::SVector{3, <:Real}

    # switch based on the type of `model`
    if typeof(model) == RigidBodyModel
        angularvelocity = RigidBody.update_angularvelocity(model, currentTime, angularvelocity, Tsampling, currentbodyframe, distinput, ctrlinput)
    elseif typeof(model) == LinearCouplingModel
        angularvelocity = LinearCoupling.update_angularvelocity(model, currentTime, angularvelocity, Tsampling, currentbodyframe, distinput, ctrlinput, straccel, strvelocity)
    else
        error("given model is invalid")
    end

    return angularvelocity
end

"""
    Base.:~(x::AbstractVector)

operator for calculating the skew-symmetric matrix. It will be used for internal calculation of the calculation of the attitude dynamics of `FlexibleSpacecraft.jl`.
"""
@inline function Base.:~(x::AbstractVector)

    if size(x, 1) != 3
        throw(DimensionMismatch("dimension of vector `x` should be 3"))
    end

    return SMatrix{3, 3, <:Real}([
        0 -x[3] x[2]
        x[3] 0 -x[1]
        -x[2] x[1] 0
    ])
end

end
