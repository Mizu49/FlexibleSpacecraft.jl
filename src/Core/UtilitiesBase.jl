module UtilitiesBase

using StaticArrays

export C1, C2, C3, yamlread2matrix

"""
    C1(theta::Real)::SMatrix

Rotational matrix for 1-axis
"""
function C1(theta::Real)::SMatrix
    return SMatrix{3, 3, <:Real}([
        1 0 0
        0 cos(theta) sin(theta)
        0 -sin(theta) cos(theta)
    ])
end

"""
    C2(theta::Real)::SMatrix

Rotational matrix for 2-axis
"""
function C2(theta::Real)::SMatrix
    return SMatrix{3, 3, <:Real}([
        cos(theta) 0 -sin(theta)
        0 1 0
        sin(theta) 0 cos(theta)
    ])
end

"""
    C3(theta::Real)::SMatrix

Rotational matrix for 3-axis
"""
function C3(theta::Real)::SMatrix
    return SMatrix{3, 3, <:Real}([
        cos(theta) sin(theta) 0
        -sin(theta) cos(theta) 0
        0 0 1
    ])
end


"""
    yamlread2matrix

function that converts the direct read data from YAML file into the Matrix

# Argument

* `x::AbstractVector`: direct read data from YAML
* `size::Tuple{<:Int, <:Int}`: size of the desired matrix
"""
@inline function yamlread2matrix(x::AbstractVector, size::Tuple{<:Int, <:Int})
    return Matrix(transpose(reshape(x, reverse(size))))
end

end
