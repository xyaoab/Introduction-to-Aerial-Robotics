## Proj1 Phase4 Report Xinjie Yao 20327521

#### Analysis on flying procedure

To ensure stability and smoothness, we choose position command with velocity feedforward. As each side of the sqare trajectory is just a simple linear function with respect to time, the velocity  in $x,y$ axis are constant and the accerleration in $x,y,z$ axis are all set to 0. Together with no control of the yaw angle, the waypoints are generated within each small step as $t​$ changes by 0.02.  

Due to imperfect PID tuning, the UAV can't remain at the same height when following the sqaure trajetcory. Also, the flight controller sometimes lost connection to the motors because of imcompetent soldering work and tight wiring connections at hubs. We amended the wiring connections problem to prevent future happening. Currently the trajetory has to be coded as a predetermined function of time, which doesn't allow arbitary curves. This rudimentary approach is incapable of handling complex trajectories. 

#### Trajectory tracking implementation

We simplily derive the square trajectory as a linear piecewise function and each side takes the equal amount of time to follow. Based on the $x,y$ position function, we take derivative to obtain the velocity feedforward constant of each side.

TODO: This report will be updated when we implement minimum snap trajectory. 

#### Experiment performance

After fixing the loose connection of the flight controller, the UAV could perform stable and swift square trajectory tracking with random deviation from the $z$ setting point. All corner points didn't appear to have sudden stops/sharp turnings thanks to the velocity feedward to improve smoothness. 

#### Individual contribution 

In project 1, I worked on assemebling quadrotors, making wires and routing cables, manual flying UAV but under the risk of destroying it and programed the square trajectory tracking function. 

