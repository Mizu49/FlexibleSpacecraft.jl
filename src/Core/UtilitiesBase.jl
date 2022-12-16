module UtilitiesBase

using StaticArrays

export check_size, C1, C2, C3, yamlread2matrix, dcm2quaternion, euler2dcm, quaternion2dcm, dcm2euler, quaternion2euler, euler2quaternion

"""
    check_size

check size of the vector
"""
@inline function check_size(vector::AbstractVector, givensize::Integer)::Nothing

    if size(vector, 1) != givensize
        throw(DimensionMismatch("size of given vector should be $givensize"))
    end

    return
end

"""
    check_size

check size of the matrix
"""
@inline function check_size(matrix::AbstractMatrix, givensize::Tuple{<:Integer, <:Integer})::Nothing

    if size(matrix) != givensize
        throw(DimensionMismatch("size of given matrix should be $givensize"))
    end

    return
end

"""
    Base.:~(x::AbstractVector)

operator for calculating the skew-symmetric matrix. It will be used for internal calculation of the calculation of the attitude dynamics of `FlexibleSpacecraft.jl`.
"""
@inline function Base.:~(x::AbstractVector)

    check_size(x, 3)

    return SMatrix{3, 3, <:Real}([
        0 -x[3] x[2]
        x[3] 0 -x[1]
        -x[2] x[1] 0
    ])
end

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

"""
    dcm2quaternion(dcm::Matrix{Real})::Vector{Real}

calculate quaternion from direction cosine matrix (DCM) `dcm`
"""
function dcm2quaternion(dcm::SMatrix{3, 3, <:AbstractFloat})::SVector{4, Real}

    _checkdcm(dcm)

    q = [
        sqrt(1 + dcm[1,1] - dcm[2,2] - dcm[3,3])/2,
        sqrt(1 - dcm[1,1] + dcm[2,2] - dcm[3,3])/2,
        sqrt(1 - dcm[1,1] - dcm[2,2] + dcm[3,3])/2,
        sqrt(1 + dcm[1,1] + dcm[2,2] + dcm[3,3])/2
    ]

    (maxvalue, maxindex) = findmax(q)

    if maxindex == 1
        q[2] = 0.25/q[1] * (dcm[1,2] + dcm[2,1])
        q[3] = 0.25/q[1] * (dcm[1,3] + dcm[3,1])
        q[4] = 0.25/q[1] * (dcm[2,3] - dcm[3,2])
    elseif maxindex == 2
        q[1] = 0.25/q[2] * (dcm[1,2] + dcm[2,1])
        q[3] = 0.25/q[2] * (dcm[3,2] + dcm[2,3])
        q[4] = 0.25/q[2] * (dcm[3,1] - dcm[1,3])
    elseif maxindex == 3
        q[1] = 0.25/q[3] * (dcm[3,1] + dcm[1,3])
        q[2] = 0.25/q[3] * (dcm[3,2] + dcm[2,3])
        q[4] = 0.25/q[3] * (dcm[1,2] - dcm[2,1])
    elseif maxindex == 4
        q[1] = 0.25/q[4] * (dcm[2,3] - dcm[3,2])
        q[2] = 0.25/q[4] * (dcm[3,1] - dcm[1,3])
        q[3] = 0.25/q[4] * (dcm[1,2] - dcm[2,1])
    else
        error("`maxindex` is illegal")
    end

    return SVector{4}(q)
end

"""
    function eular2dcm(euler::Union{SVector{3, <:Real}, Vector{<:Real}})::SMatrix{3, 3, <:AbstractFloat}

calculate direction cosine matrix from the vector of z-y-x eular angles.

## Argument

* `euler::Union{SVector{3, <:Real}, Vector{<:Real}}`: each element represents the rotation with z, y, x axis, respectively
"""
function euler2dcm(euler::Union{SVector{3, <:Real}, Vector{<:Real}})::SMatrix{3, 3, <:AbstractFloat}
    s1 = sin(euler[1])
    s2 = sin(euler[2])
    s3 = sin(euler[3])
    c1 = cos(euler[1])
    c2 = cos(euler[2])
    c3 = cos(euler[3])

    dcm = SMatrix{3, 3, Float64}([
        c2*c3 c2*s3 -s2;
        (-c1*s3 + s1*s2*c3) (c1*c3 + s1*s2*s3) s1*c2;
        (s1*s3 + c1*s2*c3) (-s1*c3 + c1*s2*s3) c1*c2
    ])
end

"""
    quaternion2dcm(q::Union{Vector{<:Real}, SVector{4, <:Real}})

calculates direction cosine matrix from quaternion
"""
function quaternion2dcm(q::Union{Vector{<:Real}, SVector{4, <:Real}})::SMatrix{3, 3, <:AbstractFloat}
    q2 = q.^2;

    dcm = SMatrix{3, 3}([
        (q2[1] - q2[2] - q2[3] + q2[4]) 2*(q[1]*q[2] + q[3]*q[4]) 2*(q[1]*q[3] - q[2]*q[4]);
        2*(q[1]*q[2] - q[3]*q[4]) (q2[2] - q2[1] - q2[3] + q2[4]) 2*(q[2]*q[3] + q[1]*q[4]);
        2*(q[1]*q[3] + q[2]*q[4]) 2*(q[2]*q[3] - q[1]*q[4]) q2[3] - q2[1] - q2[2] + q2[4]
    ])

    return dcm
end

"""
    dcm2euler(dcm::Union{SMatrix{3, 3, <:Real}, Matrix{<:Real}})::SVector{3, <:Real}

calculates z-y-x euler rotation angle from direction cosine matrix
"""
function dcm2euler(dcm::Union{SMatrix{3, 3, <:Real}, Matrix{<:Real}})::SVector{3, <:Real}
    _checkdcm(dcm)

    euler = SVector{3}([
        atan(dcm[2,3], dcm[3,3]),
        atan(-dcm[1,3], sqrt(dcm[2,3]^2 + dcm[3,3]^2)),
        atan(dcm[1,2], dcm[1,1])
    ])

    return euler
end

"""
    quaternion2euler(quaternion::Union{Vector{<:Real}, SVector{4, <:Real}})::SVector{3, <:Real}

calculates z-y-x euler rotation angle from quaternion
"""
function quaternion2euler(quaternion::Union{Vector{<:Real}, SVector{4, <:Real}})::SVector{3, <:Real}

    # use DCM for the calculation
    euler = dcm2euler(quaternion2dcm(quaternion))

    return euler
end

"""
    euler2quaternion(euler::Union{SVector{3, <:Real}, Vector{<:Real}})::SVector{4, Real}

calculates quaternion from z-y-x euler rotation angle
"""
function euler2quaternion(euler::Union{SVector{3, <:Real}, Vector{<:Real}})::SVector{4, Real}

    # use DCM for the calculation
    quaternion = dcm2quaternion(euler2dcm(euler))

    return quaternion
end

function _checkdcm(dcm::Union{SMatrix{3, 3, <:Real}, Matrix{<:Real}})
    if size(dcm) != (3, 3)
        throw(ArgumentError("`dcm` should be `3x3` matrix"))
    end
    return
end

"""
    Base.Math.deg2rad(rotationangle::Union{SVector{3, <:Real}, Vector{<:Real})

Convert rotation angle vector in degrees to radians
"""
function Base.Math.deg2rad(rotationangle::Union{SVector{3, <:Real}, Vector{<:Real}})

    newangle = SVector{3}([
        deg2rad(rotationangle[1])
        deg2rad(rotationangle[2])
        deg2rad(rotationangle[3])
    ])

    return newangle
end

"""
    Base.Math.rad2deg(rotationangle::Union{SVector{3, <:Real}, Vector{<:Real})

Convert rotation angle vector in radians to degrees
"""
function Base.Math.rad2deg(rotationangle::Union{SVector{3, <:Real}, Vector{<:Real}})

    newangle = SVector{3}([
        rad2deg(rotationangle[1])
        rad2deg(rotationangle[2])
        rad2deg(rotationangle[3])
    ])

    return newangle
end

end
