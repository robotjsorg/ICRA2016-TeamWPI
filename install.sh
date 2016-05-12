# go to locally installed fmrbenchmark folder
cd $FMRBENCHMARK

# make directory (analogus to fmrb_demo)
mkdir -p team_wpi-solution/src
cd team_wpi_solution/src
catkin_init_workspace

# link folders
ln -s $FMRBENCHMARK/domains/integrator_chains/dynamaestro
ln -s $FMRBENCHMARK/domains/integrator_chains/integrator_chains_msgs

tar -zxvf team_wpi.tar.gz
cd ..
catkin_make install
