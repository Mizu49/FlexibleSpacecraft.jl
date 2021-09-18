"""
    module TimeLine

module of time line of the physical quantity of spacecraft attitude dynamics

"""
module TimeLine


"""
    struct Frame(x::Vector{Real}, y::Vector{Real}, z::Vector{Real})

Struct of immutable vectors that express the coordinate frame of a certain state
"""
struct Frame
    x::Vector{Real}
    y::Vector{Real}
    z::Vector{Real}
end

"""
    mutable struct FrameArray(x::Array{Real, 2}, y::Array{Real, 2}, z::Array{Real, 2})

Mutable struct of array of time-variant coordinate frame vectors
"""
mutable struct FrameArray
    x::Array{Real, 2}
    y::Array{Real, 2}
    z::Array{Real, 2}
end

"""
    function getindex(frame::FrameArray)::Frame

Retrieve the frame vectors from the frame vectors array.

# Example

julia > framearray = FrameArray( ... )

julia > framearray[100]

Frame(
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0]
)
"""
function Base.getindex(framearray::FrameArray, index::Int)::Frame

    1 <= index <= size(framearray.x, 2) || throw(BoundsError(framearray, index))

    sampledframe = Frame(
        framearray.x[:, index],
        framearray.y[:, index],
        framearray.z[:, index]
    )

    return sampledframe
end


"""
    function Base.setindex!(framearray::FrameArray, frame::Frame, index::Int)::Nothing

Store the given `frame::Frame` value at the given index within a collection `framearray::FrameArray`.
"""
function Base.setindex!(framearray::FrameArray, frame::Frame, index::Int)::Nothing

    framearray.x[:, index] = frame.x
    framearray.y[:, index] = frame.y
    framearray.z[:, index] = frame.z

    return
end

"""
    function Base.setindex!(framearray::FrameArray, frame::Tuple{Vector, Vector, Vector}, index::Int)::Nothing

Store the given `frame::Tuple{Vector, Vector, Vector}` value at the given index within a collection `framearray::FrameArray`.
"""
function Base.setindex!(framearray::FrameArray, frame::Tuple{Vector, Vector, Vector}, index::Int)::Nothing

    framearray.x[:, index] = frame[1]
    framearray.y[:, index] = frame[2]
    framearray.z[:, index] = frame[3]

    return
end

"""
    function getframe(time, sampling_period, coordinates::FrameArray)

get a `sampledframe::Frame` matching with given `time`
"""
function getframe(time, sampling_period, frames)

    sample_step = floor(Int, time/sampling_period) + 1

    return frames[sample_step]
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

"""
    struct InitiData

Struct that consists of the initial state value of the time-variant physical amounts in simulation
"""
struct InitData

    # Spacecraft state variable
    quaternion::Vector{Real}
    angularvelocity::Vector{Real}

    bodyframes::Frame

end

"""
    struct DataTimeLine

Struct that consists of the time-variant physical amounts in simulation.
"""
struct DataTimeLine

    time::Array{Real, 1}

    # Spacecraft state variable
    quaternion::Array{Real, 2}
    angularvelocity::Array{Real, 2}
    bodyframes::FrameArray

    # Environment
    disturbance::Array{Real, 2}

    DataTimeLine(initvalues::InitData, samplingtime::Real, datanum::Int64) = begin

        # time (second)
        time = 0:samplingtime:samplingtime*(datanum-1)

        # Initialize arrays
        quaternion = init_quaternion_array(datanum, initvalues.quaternion)

        angularvelocity = init_angular_velocity_array(datanum, initvalues.angularvelocity)

        bodyframes = initframes(datanum, initvalues.bodyframes)

        disturbance = zeros(3, datanum)

        new(time, quaternion, angularvelocity, bodyframes, disturbance)
    end

end

"""
    struct SimDataSet

Struct that consists of the time-variant physical amounts in simulation at a certain time.
"""
struct SimDataSet

    time::Real

    # Spacecraft state variable
    quaternion::Vector{Real}
    angularvelocity::Vector{Real}
    bodyframe::Frame

    # Environment
    disturbance::Vector{Real}

end

"""
    function Base.getindex(simdata::DataTimeLine, idx::Int)::SimDataSet

Method that returns the `SimDataSet` from `DataTimeLine` at a certain discrete time index.
"""
function Base.getindex(simdata::DataTimeLine, idx::Int)::SimDataSet

    time = simdata.time[idx]
    angularvelocity = simdata.angularvelocity[:, idx]
    quaternion = simdata.quaternion[:, idx]
    bodyframe = simdata.bodyframes[idx]
    disturbance = simdata.disturbance[:, idx]

    return SimDataSet(time, quaternion, angularvelocity, bodyframe, disturbance)
end

end
