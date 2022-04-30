using Test

include("../../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

dcm_nominal = copy(transpose([
    0.5003982 -0.1465874  0.8532959
    0.4999999  0.8535293 -0.1465874
    -0.7068252  0.4999999  0.5003982
]))
quaternion_nominal =  [0.1913574, 0.4617177, 0.1913574, 0.8447375]
euler_nominal = [0.785, 0.785, 0.785] # 45 degree rotation in roll-pitch-yaw axis

@testset "Attitude representation conversions" begin

    @test isapprox(quaternion2dcm(quaternion_nominal), dcm_nominal, atol = 1e-4)
    @test isapprox(dcm2euler(dcm_nominal), euler_nominal, atol = 1e-4)
    @test isapprox(euler2dcm(euler_nominal), dcm_nominal, atol = 1e-4)
    @test isapprox(dcm2quaternion(dcm_nominal), quaternion_nominal, atol = 1e-4)
    @test isapprox(quaternion2euler(quaternion_nominal), euler_nominal, atol = 1e-4)
    @test isapprox(euler2quaternion(euler_nominal), quaternion_nominal, atol = 1e-4)

end
