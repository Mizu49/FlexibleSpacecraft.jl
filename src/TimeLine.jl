"""
    module TimeLine

module of time line of the physical quantity of spacecraft attitude dynamics

"""
module TimeLine


"""
    struct Coordinate(x::Vector, y::Vector, z::Vector)

Struct of immutable coordinate vectors. Use this for inertia frame (coordinate system A)
"""
struct Coordinate
    x::Vector
    y::Vector
    z::Vector
end

"""
    mutable struct CoordinateArray(x::Matrix, y::Matrix, z::Matrix)

Struct of array of time-variant coordinate vectors
"""
mutable struct CoordinateArray
    x::Matrix
    y::Matrix
    z::Matrix
end

"""
    function get_coordinate(time, sampling_period, coordinates::CoordinateArray)

get a `sample_coordinate::Coordinate` matching with given `time`
"""
function get_coordinate(time, sampling_period, coordinates::CoordinateArray)

    sample_step = floor(Int, time/sampling_period) + 1

    sample_coordinate = Coordinate(
        coordinates.x[:, sample_step],
        coordinates.y[:, sample_step],
        coordinates.z[:, sample_step]
    )

    return sample_coordinate
end

"""
    function init_angular_velocity_array(simdata_num, initital_value::Coordinate)

initialize array that contains time response of angular velocity
"""
function init_angular_velocity_array(simdata_num, initital_value::Coordinate)

    angular_velocity_array = zeros(3, simdata_num)
    angular_velocity_array[:,1] = initial_value

    return angular_velocity_array
end


"""
    function init_quaternion_array(simdata_num, initial_value::Vector[4])

initialize array that contains time response of quaternion
"""
function init_quaternion_array(simdata_num, initial_value::Vector)

    quaternion_array = zeros(4, simdata_num)
    quaternion_array[:, 1] = initial_value

    return quaternion_array
end


"""
    function init_coordinate_array(simdata_num, initial_coordinate::Coordinate)

initialize time-variant coordinate vectors
"""
function init_coordinate_array(simdata_num, initial_coordinate::Coordinate)

    coordinate_array = CoordinateVectors(
        zeros(3, simdata_num),
        zeros(3, simdata_num),
        zeros(3, simdata_num),
    )

    coordinate_array.x[:, 1] = initial_coordinate.x
    coordinate_array.y[:, 1] = initial_coordinate.y
    coordinate_array.z[:, 1] = initial_coordinate.z

    return coordinate_array
end

end
