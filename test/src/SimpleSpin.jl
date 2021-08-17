using Test

include("../../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

# Module for testing of simulation
include("SimulationTesting.jl")
using .SimulationTesting

# Test simulation script
@testset "Z-axis rotation" begin

    # inertia matrix
    inertia = diagm([1.0, 1.0, 2.0])

    # Orbit
    orbit = Orbit.CircularOrbit(4e+5, 3.9879e+14)

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
    ECI_frame = TimeLine.Frame(
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    )

    # Spacecraft-fixed frame (Body frame)
    body_frame = TimeLine.initframes(data_num, ECI_frame)

    # Angular velocity of body frame with respect to the ECI frame
    angular_velocity = TimeLine.init_angular_velocity_array(data_num, [0, 0, 0])

    quaternion = TimeLine.init_quaternion_array(data_num, [0, 0, 0, 1])

    println("Begin simulation!")
    for loopCounter = 0:data_num - 2

        # Update current time (second)
        currenttime = Tsampling * loopCounter

        # Update current attitude
        C = RigidBodyAttitudeDynamics.ECI2BodyFrame(quaternion[:, loopCounter + 1])
        body_frame.x[:, loopCounter + 1] = C * ECI_frame.x
        body_frame.y[:, loopCounter + 1] = C * ECI_frame.y
        body_frame.z[:, loopCounter + 1] = C * ECI_frame.z

        # Extract body fixed frame at current time
        currentbodyframe = TimeLine.getframe(currenttime, Tsampling, body_frame)

        # Disturbance torque
        disturbance = RigidBodyAttitudeDynamics.constant_torque()

        ##### Time evolution of the system
        # Update angular velocity
        angular_velocity[:, loopCounter + 2] = RigidBodyAttitudeDynamics.calc_angular_velocity(model, time[loopCounter + 1], angular_velocity[:, loopCounter + 1], Tsampling, currentbodyframe, disturbance)

        # Update quaternion
        quaternion[:, loopCounter + 2] = RigidBodyAttitudeDynamics.calc_quaternion(angular_velocity[:,loopCounter + 1], quaternion[:, loopCounter + 1], Tsampling)

    end
    println("Simulation is completed!")

    @test SimulationTesting.quaternion_constraint(quaternion)

    fig1 = PlotGenerator.angular_velocity(time, angular_velocity)
    display(fig1)


    fig2 = PlotGenerator.frame_gif(time, Tsampling, ECI_frame, body_frame, Tgif = 0.4, FPS = 10)
    display(fig2)

end
