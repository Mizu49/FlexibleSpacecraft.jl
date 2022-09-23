"""
    module RigidBody

module that consists variables and functions needed for the simulation of rigid body spacecraft attitude dynamics. Internal module for `FlexibleSpacecraft.jl`

# Usage
```
include("RigidBody.jl")
using .RigidBody
```
"""
module RigidBody

using StaticArrays
using ..Frames, ..DynamicsBase

export RigidBodyModel

"""
    struct RigidBodyModel

Data container of rigid body spacecraft model. Used to specify and configure the parameter settings for simulation and control model in `FlexibleSpacecraft.jl`

## Fields of struct `RigidBodyModel`

* `inertia::SMatrix{3, 3, <:Real}`: Inertia matrix of spacecraft platform
"""
struct RigidBodyModel
    # Inertia Matrix
    inertia::SMatrix{3, 3, <:Real}

    RigidBodyModel(inertia::Union{Matrix{<:Real}, SMatrix{3, 3, <:Real}}) = begin

        if size(inertia) != (3, 3)
            throw(DimensionMismatch("dimension of `inertia` is invalid, it should be 3x3."))
        end

        inertiamat = SMatrix{3, 3}(inertia)
        return new(inertiamat)
    end
end

# Equation of dynamics
"""
    function _calc_differential_dynamics(model::RigidBodyModel, currentTime::Real, angularvelocity::SVector{3, <:Real}, current_body_frame::StaticArrays.SMatrix{3, 3, Real, 9}, disturbance::Vector{<:Real})

Get the differential of equation of dynamics. Internal function for module `RigidBody`

# Arguments
- model::RigidBodyModel
- currentTime: current time of system [s]
- angularvelocity: angular velocity of body frame with respect to ECI frame [rad/s]
- current_body_frame: current body frame [b1 b2 b3]

# return
- differential: differential of equation of motion
"""
function _calc_differential_dynamics(
    model::RigidBodyModel,
    currentTime::Real,
    angularvelocity::SVector{3, <:Real},
    current_body_frame::StaticArrays.SMatrix{3, 3, <:Real, 9},
    disturbance::AbstractVector{<:Real},
    ctrlinput::AbstractVector{<:Real}
    )::SVector{3, <:Real}

    # calculate differential of equation of motion
    differential = SVector{3}(inv(model.inertia) * (ctrlinput + disturbance - current_body_frame' * model.inertia * ~(angularvelocity) * current_body_frame * current_body_frame' * angularvelocity))

    return differential
end

"""
    function update_angularvelocity(model::RigidBodyModel, currentTime::Real, angularvelocity::Union{Vector{<:Real}, SVector{3, <:Real}}, Tsampling::Real, currentbodyframe::Frame, disturbance::Union{Vector{<:Real}, SVector{3, <:Real}})::SVector{3, <:Real}

calculate angular velocity at next time step using 4th order Runge-Kutta method
"""
function update_angularvelocity(
    model::RigidBodyModel,
    currentTime::Real,
    angularvelocity::AbstractVector{<:Real},
    Tsampling::Real,
    currentbodyframe::Frame,
    disturbance::AbstractVector{<:Real},
    ctrlinput::AbstractVector{<:Real}
    )::SVector{3, <:Real}

    # define body frame matrix from struct `Frame`
    bodyframematrix = SMatrix{3, 3}(hcat(currentbodyframe.x, currentbodyframe.y, currentbodyframe.z))

    k1 = _calc_differential_dynamics(model, currentTime              , angularvelocity                   , bodyframematrix, disturbance, ctrlinput)
    k2 = _calc_differential_dynamics(model, currentTime + Tsampling/2, angularvelocity + Tsampling/2 * k1, bodyframematrix, disturbance, ctrlinput)
    k3 = _calc_differential_dynamics(model, currentTime + Tsampling/2, angularvelocity + Tsampling/2 * k2, bodyframematrix, disturbance, ctrlinput)
    k4 = _calc_differential_dynamics(model, currentTime + Tsampling  , angularvelocity + Tsampling   * k3, bodyframematrix, disturbance, ctrlinput)

    nextOmega = angularvelocity + Tsampling/6 * (k1 + 2*k2 + 2*k3 + k4)

    return nextOmega
end

end
