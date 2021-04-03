struct Point
    x::Int
    y::Int
end

struct Point2
    x::Float64
    y::Float64
end

function distance(p::Point)
    sqrt(p.x^2 + p.y^2)
end

function distance(p::Point2)
    sqrt(p.x^2 + p.y^2)
end

p = Point(2, 3)

result1 = distance(p)
println(result1)

p2 = Point2(3.5, 4.6)

result2 = distance(p2)
println(result2)

plot(result1, result2)