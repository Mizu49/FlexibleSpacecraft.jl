"""
    module Disturbance

Module that includes functions related to the calculation of disturbance to the system
"""
module Disturbance

using LinearAlgebra:norm

"""
    function constant_torque(constant_torque::Vector)

Function that returns given constant torque vector
"""
function constant_torque(constant_torque::Vector)

    return constant_torque

end


"""
    function gravity_gradient_torque(inertia, orbit_angular_velocity, C_ECI2Body, C_ECI2LVLH, LVLHframe_z)

Function that returns gravity gradient torque

# Arguments

* Inertia matrix
* Angular velocity of the orbit
* Transfromation matrix from ECI frame to body frame
* Transformation matrix from ECI frame to LVLH frame
* Z-vector of LVLH frame

"""
function gravity_gradient_torque(inertia, orbit_angular_velocity, C_ECI2Body, C_ECI2LVLH, LVLHframe_z)

    # Transformation matrix from LVLH to spacecraft body frame
    C = C_ECI2Body * inv(C_ECI2LVLH)

    # calculate nadir vector
    nadir_vector = C * LVLHframe_z

    # make it unit vector
    nadir_vector = 1/norm(nadir_vector) .* nadir_vector

    nadir_skew = [
         0 -nadir_vector[3]  nadir_vector[2]
         nadir_vector[3] 0  -nadir_vector[1]
        -nadir_vector[2]  nadir_vector[1] 0
    ]

    torque_vector = 3*orbit_angular_velocity^2 * nadir_skew * inertia * nadir_vector;

    return torque_vector
end

end
