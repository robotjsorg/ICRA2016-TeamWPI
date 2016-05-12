![Team WPI Integrator Chains Screenshots](https://raw.githubusercontent.com/jmcmahon443/team_wpi/master/team_wpi_screenshots.png?token=ABFAZZH5YBV_b0xQE6uMJPzYl9E4Q691ks5XPh6BwA%3D%3D)

# Team WPI Integrator Chains
A submission to the FMRBenchmark contest by California Institute of Technology <http://fmrchallenge.org/> at ICRA 2016. Team WPI submits two controllers to the Scaling Chains of Integrators domain, `wpi_solution_01` and `wpi_solution_02`

Rohit Sheth, Nishan Srishankar, Joseph McMahon, Professor Jie Fu

Please see FMRB's Github (https://github.com/fmrchallenge/fmrbenchmark) and its installation and usage documentation at (http://docs.fmrchallenge.org/en/latest/integrator_chains.html)

## Instructions if FMRB is already installed
1. Extract files from `team_wpi.tar.gz` or `team_wpi.zip`
2. Move the folder `team_wpi` into your instance of FMRB `integrators_workspace/src` folder created in step 2.2.2 of the documentation. Alternatively, symlink the `team_wpi` folder into `integrators_workspace/src` created in step 2.2.2 of the documentation
3. Rebuild the ROS package using `catkin_make install` in the `integrators_workspace` folder
4. Open another terminal window in `integrators_workspace` and source both windows `source install/setup.bash`
5. Run the FMRB trial runner in one window with `python $FMRBENCHMARK/domains/integrator_chains/trial-runner.py -l -f mydata.json src/sci_concrete_examples/trialconf/mc-small-out3-order3.json`
6. Run `roslaunch team_wpi_integrator_chains wpi_solution_01.launch` in another window. The second controller can be run by using `wpi_solution_02.launch` instead

## 2D and 3D overlay plots (as shown in screenshot)
A 2D and 3D plotting tool `tdstat_2d_3d_overlay.py` was created that overlays the generated paths with the goal and obstacle regions in 2D or 3D

To use it for 2D overlays, run this from your `integrators_workspace` folder `python $TEAM_WPI/tdstat_2d_3d_overlay.py -t 0 --traj --state "0,1" mydata.json` where `$TEAM_WPI` is the folder path to this GitHub repo on your machine

To use it for 3D overlays, run this instead `python $TEAM_WPI/tdstat_2d_3d_overlay.py -t 0 --traj --state "0,1,2" mydata.json`
