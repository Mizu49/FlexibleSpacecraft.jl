# Attitude disturbance calculation features for gravity gradient disturbance

# Gravity constant of the earth G x M_earth (m^3/s^2)
const μ = 3.986e14

# Diameter of the Earth (m)
const R_earth = 6371e3


"""
    _gravity_gradient

"""
function _gravity_gradient(
    attitudemodel,
    C_ECI3BRF::SMatrix{3, 3},
    nadir_earth::SVector{3},
    altitude::Real
    )::SVector{3}

    # inertia of the spacecraft considering the attitude
    I = attitudemodel.inertia

    # direction of the Earth with respect to the body reference frame
    n = nadir_earth

    # distance from center of the Earth
    R = R_earth + altitude

    # calculate the torque vector in ECI frame
    torque_vector = SVector{3}(3 * μ / R^3 * ~(n) * I * n)

    return torque_vector
end
