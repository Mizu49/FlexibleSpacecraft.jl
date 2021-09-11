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
    model = RigidBody.DynamicsModel(inertia)

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
    for loopCounter = 0:data_num - 1

        # Update current time (second)
        currenttime = Tsampling * loopCounter

        # Update current attitude
        C = RigidBody.ECI2BodyFrame(quaternion[:, loopCounter + 1])
        currentbodyframe = (C * ECI_frame.x, C * ECI_frame.y, C * ECI_frame.z)

        body_frame[loopCounter + 1] = currentbodyframe

        # Disturbance torque
        disturbance = Disturbance.constant_torque([0,0,0.02])

        ##### Time evolution of the system
        if loopCounter != data_num - 1

            # Update angular velocity
            angular_velocity[:, loopCounter + 2] = RigidBody.calc_angular_velocity(model, time[loopCounter + 1], angular_velocity[:, loopCounter + 1], Tsampling, currentbodyframe, disturbance)

            # Update quaternion
            quaternion[:, loopCounter + 2] = RigidBody.calc_quaternion(angular_velocity[:,loopCounter + 1], quaternion[:, loopCounter + 1], Tsampling)

        end

    end
    println("Simulation is completed!")

    @test SimulationTesting.quaternion_constraint(quaternion)

    fig1 = PlotRecipe.angular_velocity(time, angular_velocity)
    display(fig1)


    fig2 = PlotRecipe.frame_gif(time, Tsampling, ECI_frame, body_frame, Tgif = 0.4, FPS = 10)
    display(fig2)

end
