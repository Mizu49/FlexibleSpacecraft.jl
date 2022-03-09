using Test

include("../../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

dcm_nominal = diagm([1, 1, 1])
quaternion_nominal = [0, 0, 0, 1]
euler_nominal = [0, 0, 0]

@testset "Attitude representation conversions" begin

    @test isapprox(quaternion2dcm(quaternion_nominal), dcm_nominal, atol = 1e-7)
    @test isapprox(dcm2euler(dcm_nominal), euler_nominal, atol = 1e-7)
    @test isapprox(euler2dcm(euler_nominal), dcm_nominal, atol = 1e-7)
    @test isapprox(dcm2quaternion(dcm_nominal), quaternion_nominal, atol = 1e-7)
    @test isapprox(quaternion2euler(quaternion_nominal), euler_nominal, atol = 1e-7)
    @test isapprox(euler2quaternion(euler_nominal), quaternion_nominal, atol = 1e-7)

end
