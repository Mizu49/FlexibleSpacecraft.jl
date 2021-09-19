using Test

include("../../../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

@testset begin

    distconfig = DisturbanceConfig()

    torque = disturbanceinput(distconfig)

    @test typeof(torque) == Vector{Float64}

    println(torque)

end

