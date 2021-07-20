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

    # Disturbance disturbance
    disturbance = [0.0, 0.0, 0.0]

    # Dynamics model (mutable struct)
    model = RigidBodyAttitudeDynamics.DynamicsModel(inertia, disturbance)

    # Sampling period of simulation (second)
    Ts = 1e-2

    # Time length of simulation (second)
    simulation_time = 60

    # 時刻
    time = 0:Ts:simulation_time

    # Numbers of simulation data
    simu_data_num = round(Int, simulation_time/Ts) + 1;

    # Coordinate system of a
    coordinateA = TimeLine.Coordinate(
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    )

    # Coordinate system of b
    coordinateB = TimeLine.init_coordinate_array(simu_data_num, coordinateA)

    omegaBA = TimeLine.init_angular_velocity_array(simu_data_num, [0, 0, 1])

    quaternion = TimeLine.init_quaternion_array(simu_data_num, [0, 0, 0, 1])

    for loopCounter = 0:simu_data_num - 2

        # println(loopCounter + 1)

        # Extract body fixed frame at current time step
        currentCoordB = hcat(coordinateB.x[:,loopCounter + 1] , coordinateB.y[:,loopCounter + 1], coordinateB.z[:,loopCounter + 1])

        omegaBA[:, loopCounter + 2] = RigidBodyAttitudeDynamics.calc_angular_velocity(model, time[loopCounter + 1], omegaBA[:, loopCounter + 1], Ts, currentCoordB)

        quaternion[:, loopCounter + 2] = RigidBodyAttitudeDynamics.calc_quaternion(omegaBA[:,loopCounter + 1], quaternion[:, loopCounter + 1], Ts)

        C = RigidBodyAttitudeDynamics.calc_transformation_matrix(quaternion[:, loopCounter + 1])

        coordinateB.x[:, loopCounter + 2] = C * coordinateA.x
        coordinateB.y[:, loopCounter + 2] = C * coordinateA.y
        coordinateB.z[:, loopCounter + 2] = C * coordinateA.z

    end

    @test SimulationTesting.quaternion_constraint(quaternion)

    fig1 = PlotGenerator.angular_velocity(time, omegaBA)
    display(fig1)


    fig2 = PlotGenerator.frame_gif(time, Ts, coordinateA, coordinateB)
    display(fig2)

    bodyCoordinate = TimeLine.get_coordinate(10, Ts, coordinateB)
    fig3 = PlotGenerator.bodyframe(10, coordinateA, bodyCoordinate)
    display(fig3)

end
