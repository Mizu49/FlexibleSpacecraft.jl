"""
    NoAppendages

"""
module NoAppendages

using LinearAlgebra, StaticArrays
using ..Utilities

export NoAppendagesModel, NoAppendagesModel, updatestate

"""
    NoAppendagesModel

"""
struct NoAppendagesModel
    # Nothing
end

"""
    updatestate(model::StateSpace, Tsampling::Real, currenttime::Real, currentstate::AbstractVector, angularvelocity::AbstractVector, controlinput::Union{AbstractVector, Real}, distinput::Union{AbstractVector, Real})::AbstractVector

Calculates time evolution of the structural system with Runge-Kutta method
"""
function updatestate(
    model::NoAppendagesModel,
    Tsampling::Real,
    currenttime::Real,
    currentstate::AbstractVector,
    angularvelocity::AbstractVector,
    controlinput::Union{AbstractVector, Real},
    distinput::Union{AbstractVector, Real}
    )::AbstractVector

    return 0
end

"""
    NoAppendagesParam

struct that configures parameter setting of no flexible appendages

"""
struct NoAppendagesParam
    # Nothing
end

"""
    defmodel()

"""
function defmodel()
    # define the parameters

    params = NoAppendagesParam()
    simmodel = NoAppendagesModel()

    return (params, simmodel)
end

end
