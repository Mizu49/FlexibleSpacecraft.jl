# モジュールGreetingsの定義
module Greetings

export hello
hello(name) = println("Hello!, $name.")

goodbye(name) = println("Goodbye!, $name.")

end

# 相対パスでモジュールGreetingsを読み込む
using .Greetings

hello("Julia")

Greetings.goodbye("Julia")