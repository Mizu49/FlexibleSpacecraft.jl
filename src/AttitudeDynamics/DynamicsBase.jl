module DynamicsBase

using Reexport, StaticArrays
using ..Frames

export update_angularvelocity

include("RigidBody.jl")
@reexport using .RigidBody

include("LinearCoupling.jl")
@reexport using .LinearCoupling

TypeModels = Union{RigidBodyModel, LinearCouplingModel}

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
    straccel::AbstractVector{<:Real},
    strvelocity::AbstractVector{<:Real}
    )::SVector{3, <:Real}

    # switch based on the type of `model`
    if typeof(model) == RigidBodyModel
        angularvelocity = RigidBody.update_angularvelocity(model, currentTime, angularvelocity, Tsampling, currentbodyframe, distinput)
    elseif typeof(model) == LinearCouplingModel
        angularvelocity = LinearCoupling.update_angularvelocity(model, currentTime, angularvelocity, Tsampling, currentbodyframe, distinput, straccel, strvelocity)
    else
        error("given model is invalid")
    end

    return angularvelocity
end

end
