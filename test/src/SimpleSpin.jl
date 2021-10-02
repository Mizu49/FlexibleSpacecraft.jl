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

    # Dynamics model (mutable struct)
    model = RigidBody.RigidBodyModel(inertia)

    # Sampling period of simulation (second)
    Tsampling = 1e-2
    # Time length of simulation (second)
    simulation_time = 60

    # Earth-Centered frame (constant value)
    ECI_frame = Frame(
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

    # define a orbit info
    orbitinfo = Orbit.OrbitInfo(Orbit.OrbitalElements(111.8195, 51.6433, 421e3, 0.0001239, 241.3032, 212.0072), ECI_frame)

    distconfig = DisturbanceConfig()

    println("Begin simulation!")
    # run simulation
    @time (simdata, orbitdata) = runsimulation(model, ECI_frame, initvalue, orbitinfo, distconfig, simulation_time, Tsampling)

    println("Completed!")

    @test SimulationTesting.quaternion_constraint(simdata.quaternion)

    fig1 = PlotRecipe.angular_velocity(simdata.time, simdata.angularvelocity)
    display(fig1)


    fig2 = PlotRecipe.frame_gif(simdata.time, Tsampling, ECI_frame, simdata.bodyframes, Tgif = 0.8, FPS = 8)
    display(fig2)

end
