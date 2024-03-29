module Frames

using StaticArrays

export Frame, UnitFrame, ECI_frame, initframes, getframe, ECI2BodyFrame

"""
    struct Frame(x::Vector{Real}, y::Vector{Real}, z::Vector{Real})

Struct of immutable vectors that express the coordinate frame of a certain state
"""
struct Frame
    x::SVector{3, Real}
    y::SVector{3, Real}
    z::SVector{3, Real}
end

const UnitFrame = Frame([1, 0, 0], [0, 1, 0], [0, 0, 1])

"""
    ECI_frame

Constant variable that specifies ECI frame
"""
const ECI_frame = Frame([1, 0, 0], [0, 1, 0], [0, 0, 1])
"""
    Base.:-(a::Frame, b::Frame)::Frame

Subtraction operator for struct `Frame`.
"""
Base.:-(a::Frame, b::Frame)::Frame = Frame(a.x - b.x, a.y - b.y, a.z - b.z)

"""
    initframes(datanum, initial_coordinate::Frame)

initialize data container of time-variant coordinate frame
"""
function initframes(datanum, initialframe::Frame)

    frames = [Frame(zeros(3), zeros(3), zeros(3)) for _ in 1:datanum]
    frames[1] = initialframe

    return frames
end

"""
    getframe(time, sampling_period, coordinates::FrameArray)

get a `sampledframe::Frame` matching with given `time`
"""
function getframe(time, sampling_period, frames)

    sample_step = floor(Int, time/sampling_period) + 1

    return frames[sample_step]
end

"""
    ECI2BodyFrame

Calculate the transformation matrix from ECI frame to spacecraft body-fixed frame.

# Arguments
- `q::SVector{4, Float64}`: vector of the quaternion

# Return
- `C_ECI2BRF::SMatrix{3, 3, Float64}`: transformation matrix
"""
function ECI2BodyFrame(q::SVector{4, Float64})::SMatrix{3, 3, Float64}

    C_ECI2BRF = SMatrix{3, 3, Float64}([
        q[1]^2 - q[2]^2 - q[3]^2 + q[4]^2  2*(q[1]*q[2] + q[3]*q[4])          2*(q[1]*q[3] - q[2]*q[4])
        2*(q[2]*q[1] - q[3]*q[4])          q[2]^2 - q[3]^2 - q[1]^2 + q[4]^2  2*(q[2]*q[3] + q[1]*q[4])
        2*(q[3]*q[1] + q[2]*q[4])          2*(q[3]*q[2] - q[1]*q[4])          q[3]^2 - q[1]^2 - q[2]^2 + q[4]^2
    ])

    return C_ECI2BRF
end

"""
    Base. :*(C::Union{SMatrix{3, 3, <:Real}, Matrix{<:Real}}, refframe::Frame)::Frame

Calculate the transformed frame with transformation matrix `C` with respect to `refframe`
"""
function Base. :*(C::Union{SMatrix{3, 3, <:Real}, Matrix{<:Real}}, refframe::Frame)::Frame
    return Frame(
        C * refframe.x,
        C * refframe.y,
        C * refframe.z
    )
end

transformframe(C::Union{SMatrix{3, 3, <:Real}, Matrix{<:Real}}, refframe::Frame)::Frame = C * refframe

end
