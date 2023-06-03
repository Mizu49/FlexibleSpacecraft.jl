"""
    ConstrainedModeling

submodule that accommodate the implementation of the linear model for the attitude structure coupling
"""
module ConstrainedModeling

using StaticArrays
using ..Frames, ..DynamicsBase

export ConstrainedModel

"""
    struct ConstrainedModel

Data container of spacecraft model attitude dynamics model with the linear attitude-structure coupling. Used to specify and configure the parameter settings for simulation and control model in `FlexibleSpacecraft.jl`

# Fields of struct `ConstrainedModel`

* `inertia::SMatrix{3, 3, Float64}`: Inertia matrix of spacecraft platform
* `Dcplt::SMatrix{Float64}`: coefficient matric for the coupling dynamics with the structural motion. The size of this matrix is 3 x (dimstructure)

"""
struct ConstrainedModel
    # Inertia Matrix
    inertia::SMatrix{3, 3, Float64}

    # coefficient matrix for the coupling dynamics
    coupling::SMatrix

    # counstructor for `ConstrainedModel`
    ConstrainedModel(inertia::AbstractMatrix{<:Real}, couplingmat::AbstractMatrix{<:Real}, dimstructurestate::Int) = begin

        if size(inertia) != (3, 3)
            throw(DimensionMismatch("dimension of `inertia` is invalid, it should be 3x3."))
        end

        inertiamat = SMatrix{3, 3}(inertia)
        coupling = SMatrix{3, dimstructurestate}(couplingmat)

        return new(inertiamat, coupling)
    end
end

"""
    function _calc_differential_dynamics

Get the differential of equation of dynamics. Internal function for module `ConstrainedModeling.jl`

# Arguments

* `model::ConstrainedModel`: attitude dynamics model
* `currentTime::Real`: current time
* `angularvelocity::SVector{3, Float64}`: angular velocity vector
* `distinput::SVector{3, Float64}`: disturbance torque input vector
* `ctrlinput::SVector{3, Float64}`: control torque input vector
* `straccel::SVector{Float64}`: acceleration of the structural response of the flexible appendages
* `strvelocity::AbstractVector{<:Real}`: velocity of the structural response of the flexible appendages

"""
function _calc_differential_dynamics(
    model::ConstrainedModel,
    currentTime::Real,
    angularvelocity::SVector{3, Float64},
    distinput::SVector{3, Float64},
    ctrlinput::SVector{3, Float64},
    straccel::SVector,
    strvelocity::SVector
    )::SVector{3, Float64}

    # time-variant inertia
    I = model.inertia

    # time-variant coupling term
    D = model.coupling

    # calculate differential of equation of motion
    differential = inv(I) * (
        - ~(angularvelocity) * I * angularvelocity
        # structual coupling term
        - D * straccel
        - ~(angularvelocity) * D * strvelocity
        # input terms
        + ctrlinput # control input torque
        + distinput # disturbance torque
    )

    return differential
end

"""
    update_angularvelocity

calculate angular velocity at next time step using 4th order Runge-Kutta method

# Arguments

* `model::ConstrainedModel`: attitude dynamics model
* `currentTime::Real`: current time
* `angularvelocity::SVector{3, Float64}`: angular velocity vector
* `distinput::SVector{3, Float64}`: disturbance torque input vector
* `straccel::SVector`: acceleration of the structural response of the flexible appendages
* `strvelocity::SVector`: velocity of the structural response of the flexible appendages

"""
function update_angularvelocity(
    model::ConstrainedModel,
    currentTime::Real,
    angularvelocity::SVector{3, Float64},
    Tsampling::Real,
    distinput::SVector{3, Float64},
    ctrlinput::SVector{3, Float64},
    structure2atttiude::NamedTuple
    )::SVector{3, Float64}

    # map info on flexible appendages
    straccel = structure2atttiude.accel
    strvelocity = structure2atttiude.velocity

    k1 = _calc_differential_dynamics(model, currentTime              , angularvelocity                   , distinput, ctrlinput, straccel, strvelocity)
    k2 = _calc_differential_dynamics(model, currentTime + Tsampling/2, angularvelocity + Tsampling/2 * k1, distinput, ctrlinput, straccel, strvelocity)
    k3 = _calc_differential_dynamics(model, currentTime + Tsampling/2, angularvelocity + Tsampling/2 * k2, distinput, ctrlinput, straccel, strvelocity)
    k4 = _calc_differential_dynamics(model, currentTime + Tsampling  , angularvelocity + Tsampling   * k3, distinput, ctrlinput, straccel, strvelocity)

    updated_angularvelocity = angularvelocity + Tsampling/6 * (k1 + 2*k2 + 2*k3 + k4)

    return updated_angularvelocity
end

"""
    calc_angular_momentum

calculate the angular momentum of the attitude motion

# Arguments

* `model::ConstrainedModel`: dynamics model of the attitude motion
* `angularvelocity::SVector{3, Float64}`: angular velocity of the attitude motion
"""
function calc_angular_momentum(model::ConstrainedModel, angular_velocity::SVector{3, Float64})::SVector{3, Float64}

    momentum = model.inertia * angular_velocity

    return momentum
end


end
