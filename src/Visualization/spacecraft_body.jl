
mutable struct SpacecraftBody
    faces::SMatrix{12, 3}
    points::Vector{<:Point3}
end

Base.:*(C::SMatrix{3, 3, <:Real}, points::AbstractVector{Point{Dim, T}}) where {Dim, T} = map(point -> Point{Dim, T}(C * point) , points)

const label_positions = [
    Point3{Float64}(1.5, 0.0, 0.0),
    Point3{Float64}(-1.5, 0.0, 0.0),
    Point3{Float64}(0.0, 1.5, 0.0),
    Point3{Float64}(0.0, -1.5, 0.0),
    Point3{Float64}(0.0, 0.0, 1.5),
    Point3{Float64}(0.0, 0.0, -1.5),
]

const label_texts = [
    "FWD",
    "AFT",
    "PRT",
    "STB",
    "ZNT",
    "NDR"
]

function get_spacecraft_polygon()

    faces = [
        1 2 3
        3 4 1
        1 2 6
        1 5 6
        1 4 5
        4 5 8
        3 4 7
        4 7 8
        5 6 7
        5 7 8
        2 3 6
        3 6 7
    ]

    points = [
        Point3{Float32}([ 0.5,  0.5,  0.5]),
        Point3{Float32}([-0.5,  0.5,  0.5]),
        Point3{Float32}([-0.5, -0.5,  0.5]),
        Point3{Float32}([ 0.5, -0.5,  0.5]),
        Point3{Float32}([ 0.5,  0.5, -0.5]),
        Point3{Float32}([-0.5,  0.5, -0.5]),
        Point3{Float32}([-0.5, -0.5, -0.5]),
        Point3{Float32}([ 0.5, -0.5, -0.5])
    ]

    spacecraft = SpacecraftBody(faces, points)

    return spacecraft
end
