# Developer's Guideline

Thank you for your contributions to our project!

このガイドラインでは，`FlexibleSpacecraft.jl`の開発の参加方法を共有します．

## Quick Start Guide

### 開発環境の用意

1. [Docker Desktop](https://www.docker.com/products/docker-desktop)をインストールする．
2. [VS Code](https://code.visualstudio.com/)をインストールする．
3. VS Codeに拡張機能 [Remote Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) をインストールする．

### 開発リポジトリの用意

1. [オリジナルのリポジトリ](https://github.com/Mizu49/FlexibleSpacecraft.jl)をフォークする．
2. ローカルにフォークしたリポジトリをクローンする．
3. VS Codeでローカルのリポジトリを開く．
4. コマンドパレットから **Remote-Containers: Open Folder in Container...** を実行
5. プロジェクトのフォルダがコンテナ内で開かれる．  
   初回ビルドの時は時間がかかる可能性がある．
6. `main.jl`を開く．
7. コマンドパレットから **Julia: Execute File in REPL** を実行する．
8. プログラムが動けば成功！

## Participating development
### Branching Model

We employ the following branching model. Please try to follow this!

| Branch naming  | Description                                                                                                               | Branch off from: | Merge back into:                               | 
| -------------- | ------------------------------------------------------------------------------------------------------------------------- | ---------------- | ---------------------------------------------- | 
| `main`         | Latest stable release with version tag.                                                                                   | None             | None                                           | 
| `dev-build`    | Latest development build. Newly developed features are merged to this branch.                                             | `main`           | `release-****` or `main` with `--no-ff` option | 
| `dev-****`     | Feature development branch. Development of new feature should be on this branch. Contributors are encouraged to use this. | `development`    | `development` with `no-ff` option              | 
| `release-****` | Preparation for next release will be done in this branch.                                                                 | `development`    | `main` and `develop` with `no-ff` option       | 
| `hotfix-****`  | Bug fix in `main`                                                                                                         | `main`           | `main` and `development`                       |  | 

`****` in branch naming is a short description about development effort in that branch. It should be lower camel case (e.g. `dev-differentialEquation`).

This branching model is inspired by [Vincent Driessen's Branching Model](https://nvie.com/posts/a-successful-git-branching-model/)
