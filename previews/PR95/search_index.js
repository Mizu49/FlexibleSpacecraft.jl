var documenterSearchIndex = {"docs":
[{"location":"development/developer's-guideline/#Developer's-Guideline","page":"Developer's Guideline","title":"Developer's Guideline","text":"","category":"section"},{"location":"development/developer's-guideline/","page":"Developer's Guideline","title":"Developer's Guideline","text":"We appreciate your contributions to our project! Please read the guideline for developers.","category":"page"},{"location":"development/developer's-guideline/#Branching-Model","page":"Developer's Guideline","title":"Branching Model","text":"","category":"section"},{"location":"development/developer's-guideline/","page":"Developer's Guideline","title":"Developer's Guideline","text":"We employ the following branching model. Please try to follow this!","category":"page"},{"location":"development/developer's-guideline/","page":"Developer's Guideline","title":"Developer's Guideline","text":"Branch naming Description Branch off from: Merge back into:\nmain Latest stable release with version tag. None None\ndev-build Latest development build. Newly developed features are merged to this branch. main release-**** or main with --no-ff option\ndev-**** Feature development branch. Development of new feature should be on this branch. Contributors are encouraged to use this. development development with no-ff option\nrelease-**** Preparation for next release will be done in this branch. development main and develop with no-ff option\nhotfix-**** Bug fix in main main main and development","category":"page"},{"location":"development/developer's-guideline/","page":"Developer's Guideline","title":"Developer's Guideline","text":"**** in branch naming is a short description of the development effort in that branch. It should be a lower camel case (e.g. dev-differentialEquation).","category":"page"},{"location":"development/developer's-guideline/","page":"Developer's Guideline","title":"Developer's Guideline","text":"This branching model is inspired by Vincent Driessen's Branching Model","category":"page"},{"location":"development/developer's-guideline/#Style-Guide","page":"Developer's Guideline","title":"Style Guide","text":"","category":"section"},{"location":"development/developer's-guideline/","page":"Developer's Guideline","title":"Developer's Guideline","text":"Please follow the official style guide of JuliaLang!","category":"page"},{"location":"development/developer's-guideline/","page":"Developer's Guideline","title":"Developer's Guideline","text":"In addition, we are using the following naming conventions.","category":"page"},{"location":"development/developer's-guideline/","page":"Developer's Guideline","title":"Developer's Guideline","text":"****2####(): function that returns transformation matrix from **** to ####.  \neg. ECI2BodyFrame() returns transformation matrix from ECI frame to Spacecraft Body Fixed frame.\nC_****2####: variable that has transformation matrix from **** to ####.","category":"page"},{"location":"development/developer's-guideline/#Documentation","page":"Developer's Guideline","title":"Documentation","text":"","category":"section"},{"location":"development/developer's-guideline/","page":"Developer's Guideline","title":"Developer's Guideline","text":"Developers are encouraged to add docstrings in code. Especially when developing a new feature, be sure to explain it in detail and hopefully example code!","category":"page"},{"location":"development/developer's-guideline/","page":"Developer's Guideline","title":"Developer's Guideline","text":"Documentation page is also good! Direct to /docs/src/ to add new page in the FlexibleSpacecraft.jl Docs!","category":"page"},{"location":"development/developer's-guideline/","page":"Developer's Guideline","title":"Developer's Guideline","text":"When using the information from external sources (like books and technical articles), be sure to add appropriate reference information. We recommend following the Reference Style and Format of the American Institute of Aeronautics and Astronautics (AIAA).","category":"page"},{"location":"docs-module/docs-DataContainers/#Data-container","page":"Data containers","title":"Data container","text":"","category":"section"},{"location":"docs-module/docs-DataContainers/","page":"Data containers","title":"Data containers","text":"Data container is the set of variables that contains all the necessary data of the simulation. We have two sub-modules for data container.","category":"page"},{"location":"docs-module/docs-DataContainers/","page":"Data containers","title":"Data containers","text":"Frames: data container for frame representation\nTimeLine: data container for the time-variant physical quantities","category":"page"},{"location":"docs-module/docs-DataContainers/#Frames","page":"Data containers","title":"Frames","text":"","category":"section"},{"location":"docs-module/docs-DataContainers/","page":"Data containers","title":"Data containers","text":"Frames is the module that handles data container for attitude frame representation. Please visit frames for notation and detailed explanation of the attitude frame representation","category":"page"},{"location":"docs-module/docs-DataContainers/","page":"Data containers","title":"Data containers","text":"Modules = [Frames]\nOrder   = [:type, :function]\nPages   = [\"Frames.jl\"]","category":"page"},{"location":"docs-module/docs-DataContainers/#FlexibleSpacecraft.Frames.Frame","page":"Data containers","title":"FlexibleSpacecraft.Frames.Frame","text":"struct Frame(x::Vector{Real}, y::Vector{Real}, z::Vector{Real})\n\nStruct of immutable vectors that express the coordinate frame of a certain state\n\n\n\n\n\n","category":"type"},{"location":"docs-module/docs-DataContainers/#Base.:*-Tuple{Union{Matrix{var\"#s7\"} where var\"#s7\"<:Real, StaticArrays.SMatrix{3, 3, var\"#s6\", L} where {var\"#s6\"<:Real, L}}, Frame}","page":"Data containers","title":"Base.:*","text":"Base. :*(C::Union{SMatrix{3, 3, <:Real}, Matrix{<:Real}}, refframe::Frame)::Frame\n\nCalculate the transformed frame with transformation matrix C with respect to refframe\n\n\n\n\n\n","category":"method"},{"location":"docs-module/docs-DataContainers/#Base.:--Tuple{Frame, Frame}","page":"Data containers","title":"Base.:-","text":"Base.:-(a::Frame, b::Frame)::Frame\n\nSubtraction operator for struct Frame.\n\n\n\n\n\n","category":"method"},{"location":"docs-module/docs-DataContainers/#FlexibleSpacecraft.Frames.ECI2BodyFrame-Tuple{Any}","page":"Data containers","title":"FlexibleSpacecraft.Frames.ECI2BodyFrame","text":"ECI2BodyFrame(q)\n\nCalculate the transformation matrix from ECI frame to spacecraft body-fixed frame.\n\nArguments\n\nq: quaternion\n\nReturn\n\ntransformation_matrix: transformation matrix\n\n\n\n\n\n","category":"method"},{"location":"docs-module/docs-DataContainers/#FlexibleSpacecraft.Frames.getframe-Tuple{Any, Any, Any}","page":"Data containers","title":"FlexibleSpacecraft.Frames.getframe","text":"getframe(time, sampling_period, coordinates::FrameArray)\n\nget a sampledframe::Frame matching with given time\n\n\n\n\n\n","category":"method"},{"location":"docs-module/docs-DataContainers/#FlexibleSpacecraft.Frames.initframes-Tuple{Any, Frame}","page":"Data containers","title":"FlexibleSpacecraft.Frames.initframes","text":"initframes(datanum, initial_coordinate::Frame)\n\ninitialize StructArray of time-variant coordinate frame\n\n\n\n\n\n","category":"method"},{"location":"docs-module/docs-DataContainers/#TimeLine","page":"Data containers","title":"TimeLine","text":"","category":"section"},{"location":"docs-module/docs-DataContainers/","page":"Data containers","title":"Data containers","text":"Modules = [TimeLine]\nOrder   = [:type, :function]\nPages   = [\"TimeLine.jl\"]","category":"page"},{"location":"docs-module/docs-DataContainers/#FlexibleSpacecraft.TimeLine.InitData","page":"Data containers","title":"FlexibleSpacecraft.TimeLine.InitData","text":"struct InitData\n\nStruct that consists of the initial state value of the time-variant physical amounts in simulation\n\n\n\n\n\n","category":"type"},{"location":"docs-module/docs-DataContainers/#Base.getindex-Tuple{Vector{var\"#s6\"} where var\"#s6\"<:(StaticArrays.SVector{S, T} where {S, T}), Int64, Int64}","page":"Data containers","title":"Base.getindex","text":"Base.getindex(v::Vector{<:SVector}, r::Int, datarow::Int)\n\nget an element of the v<:SVector, used for custom data container for FlexibleSpacecraft.jl\n\n\n\n\n\n","category":"method"},{"location":"docs-module/docs-DataContainers/#Base.getindex-Tuple{Vector{var\"#s6\"} where var\"#s6\"<:(StaticArrays.SVector{S, T} where {S, T}), Union{Colon, AbstractRange}, Int64}","page":"Data containers","title":"Base.getindex","text":"Base.getindex(v::Vector{<:SVector}, r::AbstractRange, datarow::Int)\n\nget a 1-D subset of the every datarow-th row of v::Vector{<:SVector} within r::AbstractRange, used for custom data container for FlexibleSpacecraft.jl\n\n\n\n\n\n","category":"method"},{"location":"docs-module/docs-DataContainers/#FlexibleSpacecraft.TimeLine._initangularvelocity-Tuple{Any, StaticArrays.SVector{3, var\"#s6\"} where var\"#s6\"<:Real}","page":"Data containers","title":"FlexibleSpacecraft.TimeLine._initangularvelocity","text":"function _initangularvelocity(simdata_num, initital_value::Vector)\n\nInitialize array that contains time response of angular velocity\n\n\n\n\n\n","category":"method"},{"location":"docs-module/docs-DataContainers/#FlexibleSpacecraft.TimeLine._initquaternion-Tuple{Any, StaticArrays.SVector{4, var\"#s6\"} where var\"#s6\"<:Real}","page":"Data containers","title":"FlexibleSpacecraft.TimeLine._initquaternion","text":"function _initquaternion(simdata_num, initial_value::Vector[4])\n\ninitialize array that contains time response of quaternion\n\n\n\n\n\n","category":"method"},{"location":"docs-module/docs-DataContainers/#FlexibleSpacecraft.TimeLine.getdataindex-Tuple{Tuple{Real, Real}, Real}","page":"Data containers","title":"FlexibleSpacecraft.TimeLine.getdataindex","text":"function getdataindex(timerange::Tuple{<:Real, <:Real}, samplingtime::Real)::Union{UnitRange{Int64}, Colon}\n\nreturns an index::::Union{UnitRange{Int64}, Colon} that corresponding to the given timerange::Tuple{<:Real, <:Real}\n\n\n\n\n\n","category":"method"},{"location":"docs-module/docs-DataContainers/#FlexibleSpacecraft.TimeLine.initsimulationdata-Tuple{Int64, InitData}","page":"Data containers","title":"FlexibleSpacecraft.TimeLine.initsimulationdata","text":"initsimulationdata(datanum::Int, initialdata::InitData)\n\nInitialize the data container for the attitude dynamics\n\n\n\n\n\n","category":"method"},{"location":"docs-CLI/#Documentation-for-the-CLI","page":"CLI","title":"Documentation for the CLI","text":"","category":"section"},{"location":"docs-CLI/","page":"CLI","title":"CLI","text":"FlexibleSpacecraft.jl offers a useful CLI tools to accelerate your development of the spacecraft. This documentation illustrates the basic settings and configurations of the CLI tool.","category":"page"},{"location":"docs-CLI/#Pre-process-of-the-simulation-parameters","page":"CLI","title":"Pre-process of the simulation parameters","text":"","category":"section"},{"location":"docs-CLI/","page":"CLI","title":"CLI","text":"You need to prepare all of the parameters for simulation in the following manner.","category":"page"},{"location":"docs-CLI/","page":"CLI","title":"CLI","text":"You need to an YAML file that specifies the locaton of the parameter setting files for the each subsystem.","category":"page"},{"location":"docs-CLI/","page":"CLI","title":"CLI","text":"spacecraft.yml: parameters for the spacecraft itself and its model formulation\norbit.yml: parameters for the orbital motion\ndisturbance.yml: parameters for the disturbance input\nsimconfig.yml: configuration files for the simulation\ninitvalue.yml: configuration of initial value for the simulation","category":"page"},{"location":"docs-CLI/","page":"CLI","title":"CLI","text":"The location of these files should be addressed in the YAML file like:","category":"page"},{"location":"docs-CLI/","page":"CLI","title":"CLI","text":"name: Test parameters # Name of the parameters setting\ndates: \"2022/04/20\" # Date (Optional)\nnotes: \"This YAML file is only for testing the CLI\" # Notes (Optional)\n\n# Specify the relative path of the parameter configuration files (Required) \n# These file locations should be declared in this file, otherwise, the software gives an error\nconfigfiles:\n    model: \"spacecraft.yml\"\n    orbit: \"orbit2.yml\"\n    disturbance: \"disturbance.yml\"\n    simconfig: \"simconfig.yml\"\n    initvalue: \"initvalue.yml\"","category":"page"},{"location":"docs-CLI/","page":"CLI","title":"CLI","text":"Save this file as YAML files like params.yml. In the following documentation, we will use params.yml.","category":"page"},{"location":"docs-CLI/#Commands-and-basic-usage","page":"CLI","title":"Commands and basic usage","text":"","category":"section"},{"location":"docs-CLI/","page":"CLI","title":"CLI","text":"evalspacecraft is the topmost basic CLI command for FlexibleSpacecraft.jl. You need to type the subcommand to specify what you want to do with FlexibleSpacecraft.jl.","category":"page"},{"location":"docs-CLI/","page":"CLI","title":"CLI","text":"$ evalspacecraft <command>","category":"page"},{"location":"docs-CLI/","page":"CLI","title":"CLI","text":"Subcommands are listed as follows:","category":"page"},{"location":"docs-CLI/","page":"CLI","title":"CLI","text":"update: update and rebuild the FlexibleSpacecraft.jl. Recommended to use this subcommand at the first time you use FlexibleSpacecraft.jl\nrun <configfilepath>: run simulation based on the given parameter settings and configurations\nclear: remove package FlexibleSpacecraft.jl","category":"page"},{"location":"docs-CLI/","page":"CLI","title":"CLI","text":"You can also use the following flags:","category":"page"},{"location":"docs-CLI/","page":"CLI","title":"CLI","text":"-h, --help: show help\n-V, --version: show version information","category":"page"},{"location":"docs-CLI/#Example","page":"CLI","title":"Example","text":"","category":"section"},{"location":"docs-CLI/","page":"CLI","title":"CLI","text":"$ evalspacecraft -V","category":"page"},{"location":"docs-CLI/","page":"CLI","title":"CLI","text":"Suppose you have the parameter setting file params.yml in your current working directory. You can run simulation with the predefined parameters with following command. ","category":"page"},{"location":"docs-CLI/","page":"CLI","title":"CLI","text":"$ evalspacecraft run params.yml --save","category":"page"},{"location":"docs-CLI/","page":"CLI","title":"CLI","text":"--save is a flag that specifies wheather to save the simulation data or not. False by default.","category":"page"},{"location":"docs-CLI/","page":"CLI","title":"CLI","text":"To test the CLI system, please try the shellscript /test/main.sh in the GitHub repository. This will help you to find out how to use our CLI system.","category":"page"},{"location":"docs-module/docs-SimulationAPI/#Simulation-API","page":"API","title":"Simulation API","text":"","category":"section"},{"location":"docs-module/docs-SimulationAPI/","page":"API","title":"API","text":"This section contains information about the API for the simulation system of FlexibleSpacecraft.jl","category":"page"},{"location":"docs-module/docs-SimulationAPI/","page":"API","title":"API","text":"Our API mainly contains the following sub-modules:","category":"page"},{"location":"docs-module/docs-SimulationAPI/","page":"API","title":"API","text":"ParameterSetting: API for parameter setting for simulation core","category":"page"},{"location":"docs-module/docs-SimulationAPI/","page":"API","title":"API","text":"The high-level API function for the simulation is runsimulation. Basically you can run your simulation by passing all the necessary arguments into function runsimulation.","category":"page"},{"location":"docs-module/docs-SimulationAPI/#runsimulation","page":"API","title":"runsimulation","text":"","category":"section"},{"location":"docs-module/docs-SimulationAPI/","page":"API","title":"API","text":"runsimulation","category":"page"},{"location":"docs-module/docs-SimulationAPI/#FlexibleSpacecraft.runsimulation","page":"API","title":"FlexibleSpacecraft.runsimulation","text":"runsimulation(model, initvalue::TimeLine.InitData, orbitinfo::Orbit.OrbitInfo, distconfig::DisturbanceConfig, simconfig::SimulationConfig)::Tuple\n\nFunction that runs simulation of the spacecraft attitude-structure coupling problem\n\nArguments\n\nmodel: Dynamics model of the system\ninitvalue::InitData: Inital value of the simulation physical states\ndistconfig::DisturbanceConfig: Disturbanve torque input configuration\nsimconfig::SimulationConfig: Simulation configuration ParameterSetting\n\nReturn\n\nReturn is tuple of (time, attitudedata, orbitdata)\n\ntime: 1-D array of the time\nattitudedata: StructArray of trajectory of the physical amount states of the spacecraft system\norbitdata: StructArray of the orbit state trajectory\n\nUsage\n\n(time, attitudedata, orbitdata) = runsimulation(model, initvalue, orbitinfo, distconfig, simconfig)\n\n\n\n\n\n","category":"function"},{"location":"docs-module/docs-SimulationAPI/#ParameterSetting","page":"API","title":"ParameterSetting","text":"","category":"section"},{"location":"docs-module/docs-SimulationAPI/","page":"API","title":"API","text":"Modules = [ParameterSetting]\nOrder   = [:type, :function]\nPages   = [\"ParameterSetting.jl\"]","category":"page"},{"location":"docs-module/docs-SimulationAPI/#FlexibleSpacecraft.ParameterSetting.SimulationConfig","page":"API","title":"FlexibleSpacecraft.ParameterSetting.SimulationConfig","text":"struct SimulationConfig\n\nstruct that contains the information about the simulation configuration\n\nfields\n\nsimulationtime::Real: time length of the simulation\nsamplingtime::Real: sampling time of the simulation\n\n\n\n\n\n","category":"type"},{"location":"docs-module/docs-SimulationAPI/#FlexibleSpacecraft.ParameterSetting.setdisturbance-Tuple{String}","page":"API","title":"FlexibleSpacecraft.ParameterSetting.setdisturbance","text":"setdisturbance(filepath::String)::DisturbanceConfig\n\nset disturbance configuration from YAML setting file\n\n\n\n\n\n","category":"method"},{"location":"docs-module/docs-SimulationAPI/#FlexibleSpacecraft.ParameterSetting.setdynamicsmodel-Tuple{String}","page":"API","title":"FlexibleSpacecraft.ParameterSetting.setdynamicsmodel","text":"setdynamicsmodel(filepath::String)\n\nLoad the YAML file configuration and construct the appropriate model for the simulation\n\n\n\n\n\n","category":"method"},{"location":"docs-module/docs-SimulationAPI/#FlexibleSpacecraft.ParameterSetting.setinitvalue-Tuple{String}","page":"API","title":"FlexibleSpacecraft.ParameterSetting.setinitvalue","text":"setinitvalue(filepath::String)::InitData\n\nDefine the inital value for simulation\n\n\n\n\n\n","category":"method"},{"location":"docs-module/docs-SimulationAPI/#FlexibleSpacecraft.ParameterSetting.setorbit-Tuple{String, Frame}","page":"API","title":"FlexibleSpacecraft.ParameterSetting.setorbit","text":"setorbit(filepath::String, ECI::Frame)::OrbitInfo\n\nLoad the YAML file configuration and construct the appropriate model for the simulation\n\n\n\n\n\n","category":"method"},{"location":"docs-module/docs-SimulationAPI/#FlexibleSpacecraft.ParameterSetting.setsimconfig-Tuple{String}","page":"API","title":"FlexibleSpacecraft.ParameterSetting.setsimconfig","text":"setsimconfig(filepath::String)::SimulationConfig\n\ninitialize the simulation configurations\n\nReturn value\n\nsimconfig::SimulationConfig\n\n\n\n\n\n","category":"method"},{"location":"dynamics/frames/#Frames","page":"Frames","title":"Frames","text":"","category":"section"},{"location":"dynamics/frames/","page":"Frames","title":"Frames","text":"We need several coordinate frames to express spacecraft attitude dynamics.","category":"page"},{"location":"dynamics/frames/#ECI-(Earth-Centered-Inertial)-Frame","page":"Frames","title":"ECI (Earth-Centered Inertial) Frame","text":"","category":"section"},{"location":"dynamics/frames/","page":"Frames","title":"Frames","text":"Inertial frame of our dynamics. Always fixed. Attitude dynamics are usually described based on this frame.","category":"page"},{"location":"dynamics/frames/","page":"Frames","title":"Frames","text":"Origin: center of the Earth\nX-axis: direction of crossing between the equator and latitude 0 degrees at the initial state\nY-axis: direction of crossing between the equator and longitude +90 degrees at the initial state\nZ-axis: direction corresponding to the Earth's rotation axis","category":"page"},{"location":"dynamics/frames/#ECEF-(Earth-Centered-Earth-fixed)-Frame","page":"Frames","title":"ECEF (Earth-Centered Earth-fixed) Frame","text":"","category":"section"},{"location":"dynamics/frames/","page":"Frames","title":"Frames","text":"Rotating coordinate frame according to the Earth's rotation. This frame is used mainly to express the ground equipment on the Earth.","category":"page"},{"location":"dynamics/frames/","page":"Frames","title":"Frames","text":"Origin: center of the Earth\nX-axis: longitude 0 degrees on Greenwich meridian\nY-axis: longitude 90 degrees east\nZ-axis: direction corresponding to the Earth's north pole","category":"page"},{"location":"dynamics/frames/#LVLH-(Local-Vertical-Local-Horizontal)-frame","page":"Frames","title":"LVLH (Local Vertical Local Horizontal) frame","text":"","category":"section"},{"location":"dynamics/frames/","page":"Frames","title":"Frames","text":"Referential frame of spacecraft on orbit. This frame describes the motion of spacecraft on orbit. And spacecraft attitude (Spacecraft-fixed frame) is expressed with respect to this frame.","category":"page"},{"location":"dynamics/frames/","page":"Frames","title":"Frames","text":"Origin: center of the spacecraft\nX-axis: direction of travel on orbit (roll axis)\nY-axis: orthogonal direction to orbit plane (pitch axis)\nZ-axis: direction to the Earth (yaw axis)","category":"page"},{"location":"dynamics/frames/#Body-frame-(Spacecraft-fixed-frame)","page":"Frames","title":"Body frame (Spacecraft-fixed frame)","text":"","category":"section"},{"location":"dynamics/frames/","page":"Frames","title":"Frames","text":"Frame that is fixed to the spacecraft body. Describing the attitude of spacecraft.","category":"page"},{"location":"dynamics/frames/","page":"Frames","title":"Frames","text":"Origin: center of the spacecraft or referential point on spacecraft\nX-axis: X-axis of the spacecraft body\nY-axis: Y-axis of the spacecraft body\nZ-axis: Z-axis of the spacecraft body","category":"page"},{"location":"docs-module/docs-PlotRecipe/#Plot-recipe","page":"Plot recipe","title":"Plot recipe","text":"","category":"section"},{"location":"docs-module/docs-PlotRecipe/","page":"Plot recipe","title":"Plot recipe","text":"FlexibleSpacecraft.jl have a post-processing features that allow you to create the plots of the simulation results.","category":"page"},{"location":"docs-module/docs-PlotRecipe/","page":"Plot recipe","title":"Plot recipe","text":"Modules = [PlotRecipe.FramePlot, PlotRecipe.PhysicalQuantity]\nOrder   = [:type, :function]\nPages   = [\"FramePlot.jl\", \"PhysicalQuantity.jl\"]","category":"page"},{"location":"docs-module/docs-PlotRecipe/#FlexibleSpacecraft.PlotRecipe.FramePlot.dispframe-Tuple{Real, Frame, Frame}","page":"Plot recipe","title":"FlexibleSpacecraft.PlotRecipe.FramePlot.dispframe","text":"function dispframe(time, refCoordinate, coordinate)\n\nGenerates the 3D figure of body fixed frame\n\n\n\n\n\n","category":"method"},{"location":"docs-module/docs-PlotRecipe/#FlexibleSpacecraft.PlotRecipe.FramePlot.framegif-Tuple{StepRangeLen, Frame, StructArrays.StructArray}","page":"Plot recipe","title":"FlexibleSpacecraft.PlotRecipe.FramePlot.framegif","text":"function frame_gif(time, Tsampling, refCoordinate, bodyCoordinateArray, Tgif = 0.4, FPS = 15)\n\nGenerates animation of frame rotation as GIF figure\n\n\n\n\n\n","category":"method"},{"location":"docs-module/docs-PlotRecipe/#FlexibleSpacecraft.PlotRecipe.PhysicalQuantity.angularvelocities-Tuple{StepRangeLen, Vector{StaticArrays.SVector{3, Float64}}}","page":"Plot recipe","title":"FlexibleSpacecraft.PlotRecipe.PhysicalQuantity.angularvelocities","text":"function angularvelocities(time::StepRangeLen, angularvelocity::Vector{StaticArrays.SVector{3, Float64}}; timerange::Tuple{<:Real, <:Real} = (0, 0))::AbstractPlot\n\nPlots angular velocity of each axis in one figure\n\n\n\n\n\n","category":"method"},{"location":"docs-module/docs-PlotRecipe/#FlexibleSpacecraft.PlotRecipe.PhysicalQuantity.eulerangles-Tuple{StepRangeLen, Vector{StaticArrays.SVector{3, Float64}}}","page":"Plot recipe","title":"FlexibleSpacecraft.PlotRecipe.PhysicalQuantity.eulerangles","text":"eulerangles(time::StepRangeLen, eulerangle::Vector{StaticArrays.SVector{3, Float64}}; timerange::Tuple{<:Real, <:Real} = (0, 0))::AbstractPlot\n\nPlots time history of euler angles\n\n\n\n\n\n","category":"method"},{"location":"docs-module/docs-PlotRecipe/#FlexibleSpacecraft.PlotRecipe.PhysicalQuantity.quaternions-Tuple{StepRangeLen, Vector{StaticArrays.SVector{4, Float64}}}","page":"Plot recipe","title":"FlexibleSpacecraft.PlotRecipe.PhysicalQuantity.quaternions","text":"function quaternions(time::StepRangeLen, quaternion::Vector{StaticArrays.SVector{4, Float64}}; timerange::Tuple{<:Real, <:Real} = (0, 0))\n\nPlot quaternions in single plot\n\n\n\n\n\n","category":"method"},{"location":"development/environment/#Development-environment","page":"Environment","title":"Development environment","text":"","category":"section"},{"location":"development/environment/#Quick-Start-Guide","page":"Environment","title":"Quick Start Guide","text":"","category":"section"},{"location":"development/environment/#開発環境の用意","page":"Environment","title":"開発環境の用意","text":"","category":"section"},{"location":"development/environment/","page":"Environment","title":"Environment","text":"Docker Desktopをインストールする．\nVS Codeをインストールする．\nVS Codeに拡張機能 Remote Containers をインストールする．","category":"page"},{"location":"development/environment/#開発リポジトリの用意","page":"Environment","title":"開発リポジトリの用意","text":"","category":"section"},{"location":"development/environment/","page":"Environment","title":"Environment","text":"オリジナルのリポジトリをフォークする．\nローカルにフォークしたリポジトリをクローンする．\nVS Codeでローカルのリポジトリを開く．\nコマンドパレットから Remote-Containers: Open Folder in Container... を実行\nプロジェクトのフォルダがコンテナ内で開かれる．   初回ビルドの時は時間がかかる可能性がある．\nmain.jlを開く．\nコマンドパレットから Julia: Execute File in REPL を実行する．\nプログラムが動けば成功！","category":"page"},{"location":"development/environment/#Remote-Containersのインストール","page":"Environment","title":"Remote -Containersのインストール","text":"","category":"section"},{"location":"development/environment/","page":"Environment","title":"Environment","text":"Remote-Containersは，Dockerコンテナ内でVS Codeを開いて開発を行うことが出来るようにするVS Codeの拡張機能です．インストールは簡単です．","category":"page"},{"location":"development/environment/","page":"Environment","title":"Environment","text":"VS CodeのEXTENSIONS: MARKETPLACEでremote-containersを検索する．\nRemote-Containersをインストールする","category":"page"},{"location":"development/environment/#開発環境の設定ファイル","page":"Environment","title":"開発環境の設定ファイル","text":"","category":"section"},{"location":"development/environment/","page":"Environment","title":"Environment","text":"FlexibleSpacecraft.jlのリポジトリをクローンしてください．ソースコード・ドキュメンテーションおよび開発環境の構築に必要なファイルがすべてダウンロードされます．","category":"page"},{"location":"development/environment/","page":"Environment","title":"Environment","text":"開発環境はDockerコンテナの中に用意しています．.devcontainer/DockerfileにDocker Imageを作るためのDockerfileが用意されています．","category":"page"},{"location":"development/environment/","page":"Environment","title":"Environment","text":"Dockerfileは，必要なアプリやパッケージと環境設定を記述したファイルです．このファイルを基にしてコンテナが作られます．\ndevcontainer.jsonは，VS codeの拡張機能Remote-Containersを使ってコンテナを立ち上げてVS codeで開発する際の設定などを書いておくファイルです．extensionsの部分に，リモート環境で使いたいVS Codeの拡張機能を書いておくと，VS Codeでコンテナを開くときにインストールされます．今回はJuliaの拡張機能を追加します．ほかにリモート環境で使いたい拡張機能を書いておけば，インストールされます．このファイルに書く内容は，VS Codeで拡張機能のページを開いたとき，下の図の赤枠の部分に表示されます．","category":"page"},{"location":"development/environment/#Package-FlexibleSpacecraft-のアップデート","page":"Environment","title":"Package FlexibleSpacecraft のアップデート","text":"","category":"section"},{"location":"development/environment/","page":"Environment","title":"Environment","text":"開発環境を用意出来たら，まずパッケージのアップデートをする必要がある．","category":"page"},{"location":"development/environment/","page":"Environment","title":"Environment","text":"julia> ]: パッケージモードに入る\n(FlexibleSpacecraft) pkg> update: パッケージをすべてアップデート\n依存パッケージがアップデートされる．","category":"page"},{"location":"development/environment/#Reference","page":"Environment","title":"Reference","text":"","category":"section"},{"location":"development/environment/","page":"Environment","title":"Environment","text":"Developing inside a Container","category":"page"},{"location":"#FlexibleSpacecraft.jl-Documentation","page":"home","title":"FlexibleSpacecraft.jl Documentation","text":"","category":"section"},{"location":"","page":"home","title":"home","text":"FlexibleSpacecraft.jl is an Open Source Spacecraft Attitude-Structure Coupling Simulator developed in Julia Language.","category":"page"},{"location":"","page":"home","title":"home","text":"Our focus is the coupling problem of structural vibration and spacecraft attitude. The structural and attitude control systems of spacecraft have been treated as separate problems. However, the coupled structure-attitude dynamic analysis will be necessary for the next generation spacecraft with extensive and flexible structures envisioned in the future. A simulator of the structure-attitude coupling system will make it possible to verify the problem efficiently and accelerate the development of the next-generation spacecraft.","category":"page"},{"location":"","page":"home","title":"home","text":"This project is quite new and under active development for open-source release.","category":"page"},{"location":"example/#Example","page":"Example","title":"Example","text":"","category":"section"},{"location":"example/","page":"Example","title":"Example","text":"This documentation page is an example and quick start guide for the use of FlexibleSpacecraft.jl","category":"page"},{"location":"example/#Example-script-and-files","page":"Example","title":"Example script and files","text":"","category":"section"},{"location":"example/","page":"Example","title":"Example","text":"Example script main.jl is found in the /test directory. Configuration and parameter setting file is preferred for the simulation, and these files should be in YAML format. Detailed formatting for the parameter settings is found in the Parameter configurations.","category":"page"},{"location":"example/","page":"Example","title":"Example","text":"You need the following files:","category":"page"},{"location":"example/","page":"Example","title":"Example","text":"main.jl: the main script\norbit.yml: main configuration for orbital parameters\nspacecraft.yml: parameter settings for spacecraft configuration","category":"page"},{"location":"example/","page":"Example","title":"Example","text":"These files are found on the test directory in the GitHub repository. Run the main.jl, and you will get the simulation result. By default, the software provides the data set of simulation results and plots of those data. It also gives you a GIF animation of the spacecraft attitude.","category":"page"},{"location":"example/#Description-of-the-main.jl-and-UI","page":"Example","title":"Description of the main.jl and UI","text":"","category":"section"},{"location":"example/","page":"Example","title":"Example","text":"This section illustrates the user interface for running the attitude-structure coupling simulation with FlexibleSpacecraft.jl. This description is based on the contents in main.jl.","category":"page"},{"location":"example/","page":"Example","title":"Example","text":"Firstly, you need to load the module FlexibleSpacecraft into your namespace.","category":"page"},{"location":"example/","page":"Example","title":"Example","text":"using FlexibleSpacecraft","category":"page"},{"location":"example/","page":"Example","title":"Example","text":"Then you need to configure the simulation. ","category":"page"},{"location":"example/","page":"Example","title":"Example","text":"# Sampling period of simulation (second)\nTsampling = 1e-2\n# Time length of simulation (second)\nsimulation_time = 1000\n\n# Initialize the simulation configurations\n(simconfig, ECI_frame) = initsimulation(simulation_time, Tsampling)","category":"page"},{"location":"example/","page":"Example","title":"Example","text":"Then you need to define the dynamics model of spacecraft and orbit. And other parameters for simulation are defined at this point.","category":"page"},{"location":"example/","page":"Example","title":"Example","text":"# Set the dynamics model\nmodel = setdynamicsmodel(\"./test/spacecraft.yml\",)\n\n# define a orbit info\norbitinfo = initorbitinfo(\"./test/orbit.yml\", ECI_frame)\n\n# Set disturbance torque\ndistconfig = DisturbanceConfig(gravitygradient = true)","category":"page"},{"location":"example/","page":"Example","title":"Example","text":"Next, initialize the time-varying states and give all the data to the simulation API.","category":"page"},{"location":"example/","page":"Example","title":"Example","text":"# Initialize data array\ninitvalue = TimeLine.InitData(\n    [0, 0, 0, 1],\n    [0, 0, 0],\n    ECI_frame\n)","category":"page"},{"location":"example/","page":"Example","title":"Example","text":"Then you are all set! Just run runsimulation(). This function is the high-level user interface for simulation. ","category":"page"},{"location":"example/","page":"Example","title":"Example","text":"# run simulation\nprintln(\"Begin simulation!\")\n@time (time, attitudedata, orbitdata) = runsimulation(model, ECI_frame, initvalue, orbitinfo, distconfig, simconfig)\nprintln(\"Completed!\")","category":"page"},{"location":"example/","page":"Example","title":"Example","text":"Congrats! You have successfully run your simulation! Let's process your simulation data. We have covered that for you. Run quaternion_constraint() to check your result is physically making sense.","category":"page"},{"location":"example/","page":"Example","title":"Example","text":"@test Evaluation.quaternion_constraint(attitudedata.quaternion)","category":"page"},{"location":"example/","page":"Example","title":"Example","text":"Our visualization feature helps you to process your simulation effectively.","category":"page"},{"location":"example/","page":"Example","title":"Example","text":"fig1 = PlotRecipe.angularvelocities(time, attitudedata.angularvelocity)\n# fig1 = PlotRecipe.angularvelocities(time, attitudedata.angularvelocity, timerange = (0, 10))\ndisplay(fig1)\n\nfig2 = PlotRecipe.quaternions(time, attitudedata.quaternion)\ndisplay(fig2)\n\n# Plot of the body frame with respect to ECI frame\nfig3 = PlotRecipe.framegif(time, ECI_frame, attitudedata.bodyframe, Tgif = 20, FPS = 8)\ndisplay(fig3)","category":"page"}]
}