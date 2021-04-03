# モジュールGreetingsの定義
module Greetings

export hello
hello(name) = println("Hello!, $name.")

goodbye(name) = println("Goodbye!, $name.")

end

# 相対パスでモジュールGreetingsを読み込む
using .Greetings

include("modulefile.jl")
using .TestModule

hello("Julia")

Greetings.goodbye("Julia")

TestModule.testHoge(1, 1.0)

println("TestModule.a = $(TestModule.a)")