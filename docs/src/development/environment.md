# 開発環境

## 必要環境

- Docker Desktop
- VS Code
  - Remote -Containers
  - Julia Extension

## Remote -Containersのインストール

Remote-Containersは，Dockerコンテナ内でVS Codeを開いて開発を行うことが出来るようにするVS Codeの拡張機能です．インストールは簡単です．

1. VS CodeのEXTENSIONS: MARKETPLACEで`remote-containers`を検索する．
1. Remote-Containersをインストールする

## 開発環境の設定ファイル

`FlexibleSpacecraft.jl`のリポジトリをクローンしてください．ソースコード・ドキュメンテーションおよび開発環境の構築に必要なファイルがすべてダウンロードされます．

開発環境はDockerコンテナの中に用意しています．`.devcontainer/Dockerfile`にDocker Imageを作るための`Dockerfile`が用意されています．

- `Dockerfile`は，必要なアプリやパッケージと環境設定を記述したファイルです．このファイルを基にしてコンテナが作られます．
- `devcontainer.json`は，VS codeの拡張機能Remote-Containersを使ってコンテナを立ち上げてVS codeで開発する際の設定などを書いておくファイルです．`extensions`の部分に，リモート環境で使いたいVS Codeの拡張機能を書いておくと，VS Codeでコンテナを開くときにインストールされます．今回はJuliaの拡張機能を追加します．ほかにリモート環境で使いたい拡張機能を書いておけば，インストールされます．このファイルに書く内容は，VS Codeで拡張機能のページを開いたとき，下の図の赤枠の部分に表示されます．

## Reference

- [Developing inside a Container](https://code.visualstudio.com/docs/remote/containers)
