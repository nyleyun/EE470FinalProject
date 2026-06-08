Implementation and Evaluation of Autonomous Vehicle Highway Lane Following System 

Andrew Daouda, Nyle Yun

Video Presentation: [Final Project Video Link](https://youtu.be/RJuB0YfRQS8)

This project is based on this example from MATLAB: https://www.mathworks.com/help/driving/ug/highway-lane-following.html#d127e58565

The repository has two branches: main, stanley_controller
main is the original example project from MATLAB and stanley_controller is the modified version with our Stanley/PID controller implemented

The simulation results are stored in the "sim_results_default" and "sim_results_stanley" folders


The project can be opened in MATLAB using this command: 
    openProject("HighwayLaneFollowing") 
    
The following commands are used to set up and run the simulation:
    rng(0) 
    mpcverbosity("off") 
    helperSLHighwayLaneFollowingSetup("scenarioFcnName", "scenario_LFACC_03_Curve_StopnGo")
    sim("HighwayLaneFollowingTestBench") 

These commands are used to generate data plots:
    helperPlotLFLateralResults(logsout) 
    hFigLongResults = helperPlotLFLongitudinalResults(logsout, time_gap, default_spacing)
