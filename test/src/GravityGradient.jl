using Test

include("../../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

# Module for testing of simulation
include("SimulationTesting.jl")
using .SimulationTesting

# Test simulation script
@testset "Gravity-Gradient Torque" begin

    # inertia matrix
    inertia = [
        45000 -1000 300
        -1000 1200 750
        300 750 50000
    ]

    # define a circular orbit info
    circular_orbit = Orbit.CircularOrbit(6370e+3 + 400e3, 3.986e+14)

    orbit_angular_velocity = Orbit.get_angular_velocity(circular_orbit)

    speed = Orbit.get_velocity(circular_orbit)

    T = Orbit.get_timeperiod(circular_orbit, unit = "minute")

    # Orbital elements of ISS
    elem = Orbit.OrbitalElements(0, 0, 6370e+3 + 400e3, 1, 0, 0)

    # Dynamics model (mutable struct)
    model = RigidBody.RigidBodyModel(inertia)

    # Sampling period of simulation (second)
    Tsampling = 1

    # Time length of simulation (second)
    simulation_time = 5400 * 2

    # Array of time
    time = 0:Tsampling:simulation_time

    # Numbers of simulation data
    data_num = round(Int, simulation_time/Tsampling) + 1;

    # Earth-Centered frame (constant value)
    ECI_frame = Frame(
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    )

    orbit_frame = Orbit.calc_orbitalframe(elem, ECI_frame)

    # Spacecraft-fixed frame (Body frame)
    body_frame = TimeLine.initframes(data_num, ECI_frame)

    # Spacecraft-LVLH frame
    spacecraft_LVLH = TimeLine.initframes(data_num, orbit_frame)

    # Angular velocity of body frame with respect to the ECI frame
    body_angular_velocity = TimeLine.init_angular_velocity_array(data_num, [0.00, 0, 0])


    quaternion = TimeLine.init_quaternion_array(data_num, [0, 0, 0, 1])

    println("Begin simulation!")
    for loopCounter = 0:data_num - 1

        currenttime = time[loopCounter + 1]

        # update transformation matrix from orbit plane frame to radial along tract frame
        C_RAT = Orbit.OrbitalPlaneFrame2RadialAlongTrack(elem, orbit_angular_velocity, currenttime)

        # calculates transformation matrix from orbital plane frame to radial along frame
        C_ECI2LVLH = Orbit.OrbitalPlaneFrame2LVLH(C_RAT)
        spacecraft_LVLH[loopCounter + 1] = C_ECI2LVLH * ECI_frame

        # transfromation matrix from ECI to body frame
        C_ECI2Body = ECI2BodyFrame(quaternion[:, loopCounter + 1])
        body_frame[loopCounter + 1] = C_ECI2Body * ECI_frame

        # Extract body fixed frame at current time
        currentbodyframe = TimeLine.getframe(currenttime, Tsampling, body_frame)

        # Disturbance torque
        disturbance = Disturbance.gravity_gradient_torque(inertia, orbit_angular_velocity, C_ECI2Body, C_ECI2LVLH, spacecraft_LVLH.z[:, loopCounter + 1])

        # Time evolution
        if loopCounter != data_num - 1

            body_angular_velocity[:, loopCounter + 2] =  RigidBody.calc_angular_velocity(model, time[loopCounter + 1], body_angular_velocity[:, loopCounter + 1], Tsampling, currentbodyframe, disturbance)

            quaternion[:, loopCounter + 2] = RigidBody.calc_quaternion(body_angular_velocity[:,loopCounter + 1], quaternion[:, loopCounter + 1], Tsampling)

        end

    end
    println("Simulation is completed!")

    @test SimulationTesting.quaternion_constraint(quaternion)

    fig1 = PlotRecipe.angular_velocity(time, body_angular_velocity)
    display(fig1)


    fig2 = PlotRecipe.frame_gif(time, Tsampling, ECI_frame, body_frame, Tgif=400, FPS=8)
    display(fig2)

    fig3 = PlotRecipe.frame_gif(time, Tsampling, orbit_frame, spacecraft_LVLH, Tgif=400, FPS=8)
    display(fig3)

end
