module Frames

using StaticArrays, StructArrays

export Frame, RefFrame, initframes, getframe, ECI2BodyFrame

"""
    struct Frame(x::Vector{Real}, y::Vector{Real}, z::Vector{Real})

Struct of immutable vectors that express the coordinate frame of a certain state
"""
struct Frame
    x::SVector{3, Real}
    y::SVector{3, Real}
    z::SVector{3, Real}
end

const RefFrame = Frame([1, 0, 0], [0, 1, 0], [0, 0, 1])

"""
    Base.:-(a::Frame, b::Frame)::Frame

Subtraction operator for struct `Frame`.
"""
Base.:-(a::Frame, b::Frame)::Frame = Frame(a.x - b.x, a.y - b.y, a.z - b.z)

"""
    initframes(datanum, initial_coordinate::Frame)

initialize `StructArray` of time-variant coordinate frame
"""
function initframes(datanum, initialframe::Frame)

    frames = StructArray(
        Frame(zeros(3), zeros(3), zeros(3)) for _ in 1:datanum
    )

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
    ECI2BodyFrame(q)

Calculate the transformation matrix from ECI frame to spacecraft body-fixed frame.

# Arguments
- `q`: quaternion

# Return
- `transformation_matrix`: transformation matrix
"""
function ECI2BodyFrame(q)

    # Check if the quaterion satisfies its constraint
    try
        constraint = q[1]^2 + q[2]^2 + q[3]^2 + q[4]^2

    catch constraint

        if constraint < 0.995
            error("Quaternion does not satisfy constraint")
        elseif constraint > 1.005
            error("Quaternion does not satisfy constraint")
        end
    end

    transformation_matrix = [
        q[1]^2 - q[2]^2 - q[3]^2 + q[4]^2  2*(q[1]*q[2] + q[3]*q[4])          2*(q[1]*q[3] - q[2]*q[4])
        2*(q[2]*q[1] - q[3]*q[4])          q[2]^2 - q[3]^2 - q[1]^2 + q[4]^2  2*(q[2]*q[3] + q[1]*q[4])
        2*(q[3]*q[1] + q[2]*q[4])          2*(q[3]*q[2] - q[1]*q[4])          q[3]^2 - q[1]^2 - q[2]^2 + q[4]^2
    ]

    return transformation_matrix
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
