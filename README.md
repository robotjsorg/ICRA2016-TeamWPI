![SCOI-SS](https://raw.githubusercontent.com/jmcmahon443/Scaling-Chains-of-Integrators/master/SS1.png?token=ABFAZcuhfn_M0e1kLNgK9CPb5_AvvAbiks5XPTSLwA%3D%3D)

# Team WPI Integrator Chains
A submission to the FMRBenchmark contest by California Institute of Technology <http://fmrchallenge.org/> at ICRA 2016. Team WPI submits two controllers to the Scaling Chains of Integrators domain, `wpi_solution_01` and `wpi_solution_02`

Rohit Sheth, Nishan Srishankar, Joseph McMahon
Professor Jie Fu

Please see FMRB's Github (https://github.com/fmrchallenge/fmrbenchmark) and its installation and usage documentation at (http://docs.fmrchallenge.org/en/latest/integrator_chains.html)

## Instructions if FMRB is already installed
1. Download `install.sh` and run it in Terminal with `./install.sh`. Or download one of the Team WPI ROS packages (`team_wpi.tar`, `team_wpi.zip`, `team_wpi.rar`) from this Git repo and extract it
2. Move the folder `team_wpi` into your instance of FMRB `integrators_workspace/src` folder created in step 2.2.2 of the documentation. Alternatively, symlink the `team_wpi` folder into `integrators_workspace/src` created in step 2.2.2 of the documentation
3. Rebuild the ROS package using `catkin_make install` in the `integrators_workspace` folder
4. Open another terminal window in `integrators_workspace` and source both windows `source install/setup.bash`
5. Run the FMRB trial runner in one window with `python $FMRBENCHMARK/domains/integrator_chains/trial-runner.py -l -f mydata.json src/sci_concrete_examples/trialconf/mc-small-out3-order3.json`
6. Run `roslaunch team_wpi_integrator_chains wpi_solution_01.launch` in another window. The second controller can be run by using `wpi_solution_02.launch` instead

## 2D and 3D overlay plots (as shown in screenshot)
A 2D and 3D plotting tool was created that overlays the generated paths with the goal and obstacle regions in 2D or 3D: `tdstat_2d3d_overlay.py`

To use it for 2D overlays, run this from your `integrators_workspace` folder `python $TEAM_WPI/tdstat_overlay_and_3d.py -t 0 --traj --state "0,1" mydata.json` where `$TEAM_WPI` is the folder path to this GitHub repo on your machine

To use it for 3D overlays, run this instead `python $TEAM_WPI/tdstat_2d3d_overlay.py -t 0 --traj --state "0,1,2" mydata.json`
