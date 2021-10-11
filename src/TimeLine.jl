"""
    module TimeLine

module of time line of the physical quantity of spacecraft attitude dynamics

"""
module TimeLine

using StructArrays
using StaticArrays
using ..Frames

export InitData, initsimulationdata

"""
    function init_angular_velocity_array(simdata_num, initital_value::Vector)

Initialize array that contains time response of angular velocity
"""
function initangularvelocity(datanum, initialvelocity::SVector{3, <:Real})

    angularvelocitydata = [SVector(0.0, 0.0, 0.0) for _ in 1:datanum]
    angularvelocitydata[1] = initialvelocity

    return angularvelocitydata
end


"""
    function init_quaternion_array(simdata_num, initial_value::Vector[4])

initialize array that contains time response of quaternion
"""
function initquaterniondata(datanum, initialvalue::SVector{4, <:Real})

    quaterniondata = [SVector(0.0, 0.0, 0.0, 0.0) for _ in 1:datanum]
    quaterniondata[1] = initialvalue

    return quaterniondata
end

"""
    struct InitiData

Struct that consists of the initial state value of the time-variant physical amounts in simulation
"""
struct InitData

    # Spacecraft state variable
    quaternion::SVector{4, Real}
    angularvelocity::SVector{3, Real}
    bodyframe::Frame

end

"""
    initsimulationdata(datanum::Int, initialdata::InitData)

Initialize the data container for the attitude dynamics
"""
function initsimulationdata(datanum::Int, initialdata::InitData)

    return StructArray(
        quaternion = initquaterniondata(datanum, initialdata.quaternion),
        angularvelocity = initangularvelocity(datanum, initialdata.angularvelocity),
        bodyframe = initframes(datanum, initialdata.bodyframe)
    )
end

end
