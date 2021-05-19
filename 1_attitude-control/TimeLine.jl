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
    initAngularVelocity(simDataNum, initVector)

Time response of angular velocity
"""
function initAngularVelocity(simDataNum, initVector)

    angularVelocityArray = zeros(3, simDataNum)
    angularVelocityArray[:,1] = initVector;

    return angularVelocityArray
end


"""
    initQuaternion(simDataNum, initVector)

Initialize quaternion array for simulation
"""
function initQuaternion(simDataNum, initVector)

    quaternion = zeros(4, simDataNum)
    quaternion[:, 1] = initVector;
    
    return quaternion
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