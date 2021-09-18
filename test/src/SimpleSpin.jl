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
    model = RigidBody.RigidBodyModel(inertia)

    # Sampling period of simulation (second)
    Tsampling = 1e-2
    # Time length of simulation (second)
    simulation_time = 60
    # Numbers of simulation data
    data_num = floor(Int, simulation_time/Tsampling) + 1;

    # Earth-Centered frame (constant value)
    ECI_frame = TimeLine.Frame(
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    )

    # Initialize data array
    initvalue = TimeLine.InitData(
        [0, 0, 0, 1],
        [0, 0, 0],
        ECI_frame
    )

    simdata = TimeLine.DataTimeLine(initvalue, Tsampling, data_num)

    println("Begin simulation!")
    @time for loopCounter = 0:data_num - 1

        # Update current time (second)
        currenttime = simdata.time[loopCounter + 1]

        # Update current attitude
        C = RigidBody.ECI2BodyFrame(simdata.quaternion[:, loopCounter + 1])
        currentbodyframe = (C * ECI_frame.x, C * ECI_frame.y, C * ECI_frame.z)

        simdata.bodyframes[loopCounter + 1] = currentbodyframe

        # Disturbance torque
        disturbance = Disturbance.constant_torque([0,0,0.02])

        ##### Time evolution of the system
        if loopCounter != data_num - 1

            # Update angular velocity
            simdata.angularvelocity[:, loopCounter + 2] = RigidBody.calc_angular_velocity(model, simdata.time[loopCounter + 1], simdata.angularvelocity[:, loopCounter + 1], Tsampling, currentbodyframe, disturbance)

            # Update quaternion
            simdata.quaternion[:, loopCounter + 2] = RigidBody.calc_quaternion(simdata.angularvelocity[:,loopCounter + 1], simdata.quaternion[:, loopCounter + 1], Tsampling)

        end

    end
    println("Simulation is completed!")

    @test SimulationTesting.quaternion_constraint(simdata.quaternion)

    fig1 = PlotRecipe.angular_velocity(simdata.time, simdata.angularvelocity)
    display(fig1)


    fig2 = PlotRecipe.frame_gif(simdata.time, Tsampling, ECI_frame, simdata.bodyframes, Tgif = 0.8, FPS = 8)
    display(fig2)

end
