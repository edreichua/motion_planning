README for Motion Planning

Author: Edrei Chua
Created on: 01/28/2016

*********** DIRECTORY STRUCTURE ***********

There are a few important files in this directory:

Report (directory for report)
    motionplanning.pdf (detailed documentation of the code)
    motionplanning.tex (tex file)
src (directory for source code)
    > Benchmark.java
    > Environment.java
    > MazeGenerator.java
    > MotionPlanner.java
    > MotionPlannerDriver.java
    > MotionPlannerView.java
    > InformedSearchProblem.java
    > PlanarRobot.java
    > PRMPlanner.java
    > Robot.java
    > RobotArm.java
    > RRTPlanner.java
    > SearchProblem.java
    > Trajectory.java
    > Vector.java


*********** HOW TO START THE DEFAULT PROGRAM ***********

To start the program, compile all the .java files.

To run the default PRMPlanner, set Is_Planar_Robot to false and run MotionPlannerDriver
To run the default RRTPlanner, set Is_Planar_Robot to true and run MotionPlannerDriver

*********** Optimization ***********

To perform optimization for PRMPlanner, set the variable optimize to true in PRMPlanner.java
To perform optimization for RRTPlanner, set the variable optimize to true in RRTPlanner.java