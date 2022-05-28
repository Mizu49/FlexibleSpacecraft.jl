"""
    LinearCoupling

submodule that accommodate the implementation of the linear model for the attitude structure coupling
"""
module LinearCoupling

using StaticArrays
using ..Frames

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

Get the differential of equation of dynamics. Internal function for module `RigidBody`

# Arguments

"""
function _calc_differential_dynamics(model::LinearCouplingModel, currentTime::Real, angularvelocity::AbstractVector{<:Real}, current_body_frame::SMatrix{3, 3, <:Real, 9}, disturbance::AbstractVector{<:Real}, structuralinput::AbstractVector{<:Real})::SVector{3, <:Real}

    # skew matrix of angular velocity vector
    skewOmega = [
        0 -angularvelocity[3] angularvelocity[2]
        angularvelocity[3] 0 -angularvelocity[1]
        -angularvelocity[2] angularvelocity[1] 0]

    # calculate differential of equation of motion
    differential = SVector{3}(inv(model.inertia) * (disturbance - current_body_frame' * model.inertia * skewOmega * current_body_frame * current_body_frame' * angularvelocity))

    return differential
end

"""
    function update_angularvelocity(model::LinearCouplingModel, currentTime::Real, angularvelocity::Union{Vector{<:Real}, SVector{3, <:Real}}, Tsampling::Real, currentbodyframe::Frame, disturbance::Union{Vector{<:Real}, SVector{3, <:Real}})::SVector{3, <:Real}

calculate angular velocity at next time step using 4th order Runge-Kutta method
"""
function update_angularvelocity(model::LinearCouplingModel, currentTime::Real, angularvelocity::Union{Vector{<:Real}, SVector{3, <:Real}}, Tsampling::Real, currentbodyframe::Frame, disturbance::Union{Vector{<:Real}, SVector{3, <:Real}}, structuralinput::AbstractVector{<:Real})::SVector{3, <:Real}

    # define body frame matrix from struct `Frame`
    bodyframematrix = SMatrix{3, 3}(hcat(currentbodyframe.x, currentbodyframe.y, currentbodyframe.z))

    k1 = _calc_differential_dynamics(model, currentTime              , angularvelocity                   , bodyframematrix, disturbance, structuralinput)
    k2 = _calc_differential_dynamics(model, currentTime + Tsampling/2, angularvelocity + Tsampling/2 * k1, bodyframematrix, disturbance, structuralinput)
    k3 = _calc_differential_dynamics(model, currentTime + Tsampling/2, angularvelocity + Tsampling/2 * k2, bodyframematrix, disturbance, structuralinput)
    k4 = _calc_differential_dynamics(model, currentTime + Tsampling  , angularvelocity + Tsampling   * k3, bodyframematrix, disturbance, structuralinput)

    nextOmega = angularvelocity + Tsampling/6 * (k1 + 2*k2 + 2*k3 + k4)

    return nextOmega
end

end
