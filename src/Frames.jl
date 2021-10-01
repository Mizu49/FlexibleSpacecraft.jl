module Frames

export Frame, ECI2BodyFrame

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
    Base. :*(C::Matrix, refframe::Frame)::Frame

Calculate the transformed frame with transformation matrix `C` with respect to `refframe`
"""
function Base. :*(C::Matrix, refframe::Frame)::Frame
    return Frame(
        C * refframe.x,
        C * refframe.y,
        C * refframe.z
    )
end

transformframe(C::Matrix, refframe::Frame)::Frame = C * refframe

end
