"""
    LinearCoupling

submodule that accommodate the implementation of the linear model for the attitude structure coupling
"""
module LinearCoupling

using StaticArrays
using ..Frames, ..DynamicsBase

export LinearCouplingModel

"""
    struct LinearCouplingModel

Data container of spacecraft model attitude dynamics model with the linear attitude-structure coupling. Used to specify and configure the parameter settings for simulation and control model in `FlexibleSpacecraft.jl`

# Fields of struct `LinearCouplingModel`

* `inertia::SMatrix{3, 3, <:Real}`: Inertia matrix of spacecraft platform
* `Dcplt::SMatrix{<:Real}`: coefficient matric for the coupling dynamics with the structural motion. The size of this matrix is 3 x (dimstructure)

"""
struct LinearCouplingModel
    # Inertia Matrix
    inertia::SMatrix{3, 3, <:Real}

    # coefficient matrix for the coupling dynamics
    Dcplg::SMatrix

    # counstructor for `LinearCouplingModel`
    LinearCouplingModel(inertia::AbstractMatrix{<:Real}, couplingmat::AbstractMatrix{<:Real}, dimstructurestate::Int) = begin

        if size(inertia) != (3, 3)
            throw(DimensionMismatch("dimension of `inertia` is invalid, it should be 3x3."))
        end

        inertiamat = SMatrix{3, 3}(inertia)
        Dcplg = SMatrix{3, dimstructurestate}(couplingmat)

        return new(inertiamat, Dcplg)
    end
end

"""
    function _calc_differential_dynamics

Get the differential of equation of dynamics. Internal function for module `LinearCoupling.jl`

# Arguments

* `model::LinearCouplingModel`: attitude dynamics model
* `currentTime::Real`: current time
* `angularvelocity::AbstractVector{<:Real}`: angular velocity vector
* `current_body_frame::SMatrix{3, 3, <:Real, 9}`: current body frame matrix
* `distinput::AbstractVector{<:Real}`: disturbance torque input vector
* `straccel::AbstractVector{<:Real}`: acceleration of the structural response of the flexible appendages
* `strvelocity::AbstractVector{<:Real}`: velocity of the structural response of the flexible appendages

"""
function _calc_differential_dynamics(
    model::LinearCouplingModel,
    currentTime::Real,
    angularvelocity::AbstractVector{<:Real},
    current_body_frame::SMatrix{3, 3, <:Real, 9},
    distinput::AbstractVector{<:Real},
    straccel::AbstractVector{<:Real},
    strvelocity::AbstractVector{<:Real}
    )::SVector{3, <:Real}

    # calculate differential of equation of motion
    differential = SVector{3}(inv(model.inertia) * (
        distinput # disturbance torque
        - current_body_frame' * model.inertia * ~(angularvelocity) * current_body_frame * current_body_frame' * angularvelocity # attitude dynamics
        - model.Dcplg * straccel - ~(angularvelocity) * model.Dcplg * strvelocity # structural coupling
    ))

    return differential
end

"""
    update_angularvelocity

calculate angular velocity at next time step using 4th order Runge-Kutta method

# Arguments

* `model::LinearCouplingModel`: attitude dynamics model
* `currentTime::Real`: current time
* `angularvelocity::AbstractVector{<:Real}`: angular velocity vector
* `current_body_frame::SMatrix{3, 3, <:Real, 9}`: current body frame matrix
* `distinput::AbstractVector{<:Real}`: disturbance torque input vector
* `straccel::AbstractVector{<:Real}`: acceleration of the structural response of the flexible appendages
* `strvelocity::AbstractVector{<:Real}`: velocity of the structural response of the flexible appendages

"""
function update_angularvelocity(
    model::LinearCouplingModel,
    currentTime::Real,
    angularvelocity::AbstractVector{<:Real},
    Tsampling::Real,
    currentbodyframe::Frame,
    distinput::AbstractVector{<:Real},
    straccel::AbstractVector{<:Real},
    strvelocity::AbstractVector{<:Real}
    )::SVector{3, <:Real}

    # define body frame matrix from struct `Frame`
    bodyframematrix = SMatrix{3, 3}(hcat(currentbodyframe.x, currentbodyframe.y, currentbodyframe.z))

    k1 = _calc_differential_dynamics(model, currentTime              , angularvelocity                   , bodyframematrix, distinput, straccel, strvelocity)
    k2 = _calc_differential_dynamics(model, currentTime + Tsampling/2, angularvelocity + Tsampling/2 * k1, bodyframematrix, distinput, straccel, strvelocity)
    k3 = _calc_differential_dynamics(model, currentTime + Tsampling/2, angularvelocity + Tsampling/2 * k2, bodyframematrix, distinput, straccel, strvelocity)
    k4 = _calc_differential_dynamics(model, currentTime + Tsampling  , angularvelocity + Tsampling   * k3, bodyframematrix, distinput, straccel, strvelocity)

    nextOmega = angularvelocity + Tsampling/6 * (k1 + 2*k2 + 2*k3 + k4)

    return nextOmega
end

end
