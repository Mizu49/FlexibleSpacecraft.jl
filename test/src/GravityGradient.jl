using Test

include("../../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

# Module for testing of simulation
include("SimulationTesting.jl")
using .SimulationTesting

# Test simulation script
@testset "Gravity-Gradient Torque" begin

    # inertia matrix
    inertia = diagm([1.0, 1.0, 2.0])

    # define a circular orbit info
    circular_orbit = Orbit.CircularOrbit(6370e+3 + 400e3, 3.986e+14)

    orbit_angular_velocity = Orbit.get_angular_velocity(circular_orbit)

    speed = Orbit.get_velocity(circular_orbit)

    T = Orbit.get_timeperiod(circular_orbit, unit = "minute")

    # Orbital elements of ISS
    elem = Orbit.OrbitalElements(111.8195, 51.6433, 421e3, 0.0001239, 241.3032, 212.0072)

    # Dynamics model (mutable struct)
    model = RigidBodyAttitudeDynamics.DynamicsModel(inertia)

    # Sampling period of simulation (second)
    Tsampling = 1e-2

    # Time length of simulation (second)
    simulation_time = 60

    # Array of time
    time = 0:Tsampling:simulation_time

    # Numbers of simulation data
    data_num = round(Int, simulation_time/Tsampling) + 1;

    # Earth-Centered frame (constant value)
    ECI_frame = TimeLine.Coordinate(
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    )

    C_ECI2OrbitalPlaneFrame = Orbit.ECI2OrbitalPlaneFrame(elem)

    orbit_frame = TimeLine.Coordinate(
    C_ECI2OrbitalPlaneFrame * ECI_frame.x,
    C_ECI2OrbitalPlaneFrame * ECI_frame.y,
    C_ECI2OrbitalPlaneFrame * ECI_frame.z
    )


    # Spacecraft-fixed frame (Body frame)
    body_frame = TimeLine.init_coordinate_array(data_num, ECI_frame)

    # Spacecraft-LVLH frame
    spacecraft_LVLH = TimeLine.init_coordinate_array(data_num, orbit_frame)

    # Angular velocity of body frame with respect to the ECI frame
    body_angular_velocity = TimeLine.init_angular_velocity_array(data_num, [0, 0, 0])


    quaternion = TimeLine.init_quaternion_array(data_num, [0, 0, 0, 1])

    println("Begin simulation!")
    for loopCounter = 0:data_num - 2

        # Extract body fixed frame at current time step
        currentCoordB = hcat(body_frame.x[:,loopCounter + 1] , body_frame.y[:,loopCounter + 1], body_frame.z[:,loopCounter + 1])

        # update current RadialAngleTrack and LVLH frame
        RAT = Orbit.OrbitalPlaneFrame2RadialAlongTrack(elem, orbit_angular_velocity, time[loopCounter + 1])
        LVLH = Orbit.OrbitalPlaneFrame2LVLH(RAT)

        spacecraft_LVLH.x[:, loopCounter + 1] = LVLH * orbit_frame.x
        spacecraft_LVLH.y[:, loopCounter + 1] = LVLH * orbit_frame.y
        spacecraft_LVLH.z[:, loopCounter + 1] = LVLH * orbit_frame.z

        # Disturbance torque
        disturbance = RigidBodyAttitudeDynamics.gravity_gradient_torque(inertia, orbit_angular_velocity, spacecraft_LVLH.z[:, loopCounter + 1])

        body_angular_velocity[:, loopCounter + 2] = RigidBodyAttitudeDynamics.calc_angular_velocity(model, time[loopCounter + 1], body_angular_velocity[:, loopCounter + 1], Tsampling, currentCoordB, disturbance)

        quaternion[:, loopCounter + 2] = RigidBodyAttitudeDynamics.calc_quaternion(body_angular_velocity[:,loopCounter + 1], quaternion[:, loopCounter + 1], Tsampling)

        C = RigidBodyAttitudeDynamics.ECI2BodyFrame(quaternion[:, loopCounter + 1])

        body_frame.x[:, loopCounter + 2] = C * ECI_frame.x
        body_frame.y[:, loopCounter + 2] = C * ECI_frame.y
        body_frame.z[:, loopCounter + 2] = C * ECI_frame.z

    end
    println("Simulation is completed!")

    @test SimulationTesting.quaternion_constraint(quaternion)

    fig1 = PlotGenerator.angular_velocity(time, body_angular_velocity)
    display(fig1)


   # fig2 = PlotGenerator.frame_gif(time, Tsampling, ECI_frame, body_frame)
    fig2 = PlotGenerator.frame_gif(time, Tsampling, orbit_frame, spacecraft_LVLH)
    display(fig2)

end
