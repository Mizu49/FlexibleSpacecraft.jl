"""
    module TimeLine

module of time line of the physical quantity of spacecraft attitude dynamics

"""
module TimeLine


"""
    struct Frame(x::Vector, y::Vector, z::Vector)

Struct of immutable vectors that express the coordinate frame of a certain state
"""
struct Frame
    x::Vector
    y::Vector
    z::Vector
end

"""
    mutable struct FrameArray(x::Matrix, y::Matrix, z::Matrix)

Mutable struct of array of time-variant coordinate frame vectors
"""
mutable struct FrameArray
    x::Matrix
    y::Matrix
    z::Matrix
end

"""
    function getframe(time, sampling_period, coordinates::FrameArray)

get a `sampledframe::Frame` matching with given `time`
"""
function getframe(time, sampling_period, frames)

    sample_step = floor(Int, time/sampling_period) + 1

    sampledframe = Frame(
        frames.x[:, sample_step],
        frames.y[:, sample_step],
        frames.z[:, sample_step]
    )

    return sampledframe
end

"""
    function init_angular_velocity_array(simdata_num, initital_value::Vector)

Initialize array that contains time response of angular velocity
"""
function init_angular_velocity_array(simdata_num, initial_velocity::Vector)

    angular_velocity_array = zeros(3, simdata_num)
    angular_velocity_array[:,1] = initial_velocity

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
    function initframes(simdata_num, initial_coordinate::Frame)

initialize time-variant coordinate frame vectors
"""
function initframes(simdata_num, initialframe::Frame)

    frames = FrameArray(
        zeros(3, simdata_num),
        zeros(3, simdata_num),
        zeros(3, simdata_num),
    )

    frames.x[:, 1] = initialframe.x
    frames.y[:, 1] = initialframe.y
    frames.z[:, 1] = initialframe.z

    return frames
end

end
