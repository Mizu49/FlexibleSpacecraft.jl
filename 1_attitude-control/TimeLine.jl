"""
    module TimeLine

module of time-variable values.
"""
module TimeLine


"""
    mutable struct CoordinateVectors(x::Matrix, y::Matrix, z::Matrix)

Struct of array of time-variant coordinate vectors
"""
mutable struct CoordinateVectors
    x::Matrix
    y::Matrix
    z::Matrix
end

"""
    initBodyCoordinate(simDataNum, initVectors)

Initialize time-variant coordinate vectors
"""
function initBodyCoordinate(simDataNum, initVector)
    coordinate = CoordinateVectors(
        zeros(3, simDataNum),
        zeros(3, simDataNum),
        zeros(3, simDataNum),
    )

    coordinate.x[:, 1] = initVector.x
    coordinate.y[:, 1] = initVector.y
    coordinate.z[:, 1] = initVector.z

    return coordinate
end

end