module ParameterSettingBase

using YAML
using ..Frames, ..Orbit, ..RigidBody, ..LinearCoupling, ..Attitude, ..Disturbance

"""
    yamlread2matrix

function that converts the direct read data from YAML file into the Matrix

# Argument

* `x::AbstractVector`: direct read data from YAML
* `size::Tuple{<:Int, <:Int}`: size of the matrix to be converted
"""
@inline function yamlread2matrix(x::AbstractVector, size::Tuple{<:Int, <:Int})
    return Matrix(reshape(x, size)')
end

end
