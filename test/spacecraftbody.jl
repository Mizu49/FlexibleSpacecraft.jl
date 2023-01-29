include("../src/FlexibleSpacecraft.jl")
using .FlexibleSpacecraft

Makie.inline!(false)


origin = [-1.0, -1.0, -1.0]
widths = [2.0, 2.0, 2.0]

spacecraft = Rect3{Float64}(origin, widths)

fig = Figure()
ax = Axis3(
    fig[1, 1],
    # elevation = pi/6,
    # azimuth   = pi/4,
    aspect = :data,
    viewmode = :fit
    ) # 3D plotする時はAxis3で3次元の軸を作る
mesh!(ax, spacecraft, color=:yellow, linewidth=2)

display(fig)

