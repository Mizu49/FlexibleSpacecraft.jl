"""
    ConstrainedModeling

submodule that accommodate the implementation of the linear model for the attitude structure coupling
"""
module ConstrainedModeling

using StaticArrays
using ..Frames, ..DynamicsBase, ..AppendagesBase

export ConstrainedModel

"""
    struct ConstrainedModel

"""
struct ConstrainedModel{T <: Union{AbstractMatrix, Function}} <: AbstractAttitudeDynamicsModel

    ## Attitude dynamics
    # Inertia Matrix
    inertia::T

    ## Flexible appendage dynamics
    appendages_params::AbstractAppendageParameters
    appendages_model::AbstractAppendageModel

end

"""
    function _calc_differential_attitude_dynamics
"""
function _calc_differential_attitude_dynamics(
    model::ConstrainedModel,
    currentTime::Real,
    angular_velocity::SVector{3, Float64},
    attitude_disturbance::SVector{3, Float64},
    attitude_control_torque::SVector{3, Float64},
    appendage_acceleration::SVector,
    current_coupling_coefficient::SMatrix
    )::SVector{3, Float64}

    # time-variant inertia
    I = model.inertia

    # calculate differential of equation of motion
    diff_angular_velocity = inv(I) * (
        - ~(angular_velocity) * I * angular_velocity
        # structual coupling term
        - current_coupling_coefficient * appendage_acceleration
        # input terms
        + attitude_control_torque
        + attitude_disturbance
    )

    return diff_angular_velocity
end

"""
    _calc_differential_appendages
"""
function _calc_differential_appendages()

end

"""
    update_attitude_dynamics

calculate angular velocity at next time step using 4th order Runge-Kutta method

"""
function update_attitude_dynamics(
    model::ConstrainedModel,
    # attitude variables at current time
    currentTime::Real,
    Tsampling::Real,
    angular_velocity::SVector{3, Float64},
    attitude_disturbance::SVector{3, Float64},
    attitude_control_torque::SVector{3, Float64},
    )::SVector{3, Float64}

    # todo: update this to provide interface for flexible appendages
    current_coupling_coefficient = @SMatrix zeros(3, 2) # fix me
    appendage_acceleration = @SVector zeros(2) # fix me

    k1 = _calc_differential_attitude_dynamics(model, currentTime              , angular_velocity                   , attitude_disturbance, attitude_control_torque, appendage_acceleration, current_coupling_coefficient)
    k2 = _calc_differential_attitude_dynamics(model, currentTime + Tsampling/2, angular_velocity + Tsampling/2 * k1, attitude_disturbance, attitude_control_torque, appendage_acceleration, current_coupling_coefficient)
    k3 = _calc_differential_attitude_dynamics(model, currentTime + Tsampling/2, angular_velocity + Tsampling/2 * k2, attitude_disturbance, attitude_control_torque, appendage_acceleration, current_coupling_coefficient)
    k4 = _calc_differential_attitude_dynamics(model, currentTime + Tsampling  , angular_velocity + Tsampling   * k3, attitude_disturbance, attitude_control_torque, appendage_acceleration, current_coupling_coefficient)

    updated_angular_velocity = angular_velocity + Tsampling/6 * (k1 + 2*k2 + 2*k3 + k4)

    return updated_angular_velocity
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
