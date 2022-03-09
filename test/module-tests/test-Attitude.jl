using Test

include("../../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

dcm_nominal = diagm([1, 1, 1])
quaternion_nominal = [0, 0, 0, 1]
euler_nominal = [0, 0, 0]

@testset "Attitude representation conversions" begin

    @test quaternion2dcm(quaternion_nominal) == dcm_nominal
    @test dcm2euler(dcm_nominal) == euler_nominal
    @test euler2dcm(euler_nominal) == dcm_nominal
    @test dcm2quaternion(dcm_nominal) == quaternion_nominal
    @test quaternion2euler(quaternion_nominal) == euler_nominal
    @test euler2quaternion(euler_nominal) == quaternion_nominal

end
