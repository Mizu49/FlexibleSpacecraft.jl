using Test

include("../../../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

println("Code testing of `TimeLine.jl`")

initvalue = TimeLine.InitData(
    [0, 0, 0, 1],
    [1, 0, 0],
    TimeLine.Frame([1, 0, 0], [0, 1, 0], [0, 0, 1])
)

samplingtime = 0.02 # second
simulation_timelength = 100 # second
datanum = Int64(floor(simulation_timelength/samplingtime) + 1)

simdata = TimeLine.DataTimeLine(initvalue, samplingtime, datanum)

show(simdata[100])
