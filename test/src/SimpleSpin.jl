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
    ECI_frame = TimeLine.Coordinate(
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    )

    # Spacecraft-fixed frame (Body frame)
    body_frame = TimeLine.init_coordinate_array(data_num, ECI_frame)

    # Angular velocity of body frame with respect to the ECI frame
    angular_velocity = TimeLine.init_angular_velocity_array(data_num, [0, 0, 0])

    quaternion = TimeLine.init_quaternion_array(data_num, [0, 0, 0, 1])

    println("Begin simulation!")
    for loopCounter = 0:data_num - 2

        # Extract body fixed frame at current time step
        currentCoordB = hcat(body_frame.x[:,loopCounter + 1] , body_frame.y[:,loopCounter + 1], body_frame.z[:,loopCounter + 1])

        # Disturbance torque
        disturbance = RigidBodyAttitudeDynamics.gravity_gradient_torque()

        angular_velocity[:, loopCounter + 2] = RigidBodyAttitudeDynamics.calc_angular_velocity(model, time[loopCounter + 1], angular_velocity[:, loopCounter + 1], Tsampling, currentCoordB, disturbance)

        quaternion[:, loopCounter + 2] = RigidBodyAttitudeDynamics.calc_quaternion(angular_velocity[:,loopCounter + 1], quaternion[:, loopCounter + 1], Tsampling)

        C = RigidBodyAttitudeDynamics.ECI2BodyFrame(quaternion[:, loopCounter + 1])

        body_frame.x[:, loopCounter + 2] = C * ECI_frame.x
        body_frame.y[:, loopCounter + 2] = C * ECI_frame.y
        body_frame.z[:, loopCounter + 2] = C * ECI_frame.z

    end
    println("Simulation is completed!")

    @test SimulationTesting.quaternion_constraint(quaternion)

    fig1 = PlotGenerator.angular_velocity(time, angular_velocity)
    display(fig1)


    fig2 = PlotGenerator.frame_gif(time, Tsampling, ECI_frame, body_frame)
    display(fig2)

end
