# robotic-arm
Detailed kinematics analysis of a 5-DOF robotic arm (RPRRR) without spherical wrist using nonlinear optimization for inverse kinematics subjected to joint constraints. 

The methodology and background of the methods are described in depth in the attached [report](https://github.com/pcko1/robotic-arm/blob/master/robot-arm-report.pdf).

The proposed inverse kinematics solution exhibits very promising results.

#### This project has been developed purely out of personal curiosity.

## Arm Geometry
<p align="center">
  <img src="Figures/DH.PNG" alt="MLP"/>
</p>

## Denavit-Hartenberg parameters

|   i   |   θ    | d   |   a  | α      |
| :---: |:------:|:---:|:----:|:------:|
| 9     | θ9     | 0 mm| 0  mm|  90 deg|
| 10    | 90 deg | d10 | 0  mm|   0 deg|
| 11    | θ11    | 0 mm| 0  mm| -90 deg|
| 12    | θ12    | 0 mm| 10 mm| -90 deg|
| 13    | θ13    | 0 mm| 8  mm| -90 deg|

## Robot Workspace (under joint constraints)
<p align="center">
  <img src="Figures/workspace.png" alt=""/>
</p>

## Desired TCP Trajectory
<p align="center">
  <img src="Figures/spiral_trajectory.png" alt=""/>
</p>

## Inverse Kinematics Solution (joint space)
<p align="center">
  <img src="Figures/inv_kin.png" alt=""/>
</p>

## Inverse Kinematics Solution (world frame)
<p align="center">
  <img src="Figures/solver_accuracy.png" alt=""/>
</p>

## Cubic Splines Interpolation in Joint Space
<p align="center">
  <img src="Figures/cubic_splines.png" alt=""/>
</p>

## Required Tool Trajectory
<p align="center">
  <img src="Figures/tool_traj.png" alt=""/>
</p>
