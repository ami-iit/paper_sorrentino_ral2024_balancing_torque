# paper_sorrentino_ral2024_balancing_torque

<p align="center">
  <img src="https://github.com/user-attachments/assets/98c5e327-35e1-41b8-bef4-bc9e77589493" alt="paperVideoBalacingController">
</p>

## Installation

For all the following steps, open a terminal and move to a directory where to install the software.
All the following steps are meant to be executed sequentially in this terminal.


Clone this repo:

```
git clone https://github.com/ami-iit/paper_sorrentino_ral2024_balancing_torque
```

Create a `conda` environment:

```
conda env create -f environment.yaml
```

Once created, activate the `conda` environment:

```
conda activate momentumbasedtorquecontrolenv
```

Clone the [`bipedal-locomotion-framework`](https://github.com/ami-iit/bipedal-locomotion-framework) and checkout the `restoreUFK` branch of [this fork](https://github.com/isorrentino/bipedal-locomotion-framework):

```
git clone https://github.com/ami-iit/bipedal-locomotion-framework
cd bipedal-locomotion-framework
git remote add fork https://github.com/isorrentino/bipedal-locomotion-framework
git fetch fork
git checkout restoreUKF
```

Configure the `bipedal-locomotion-framework`

```
cmake -S . -B build/ \
  -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX \
  -DFRAMEWORK_COMPILE_PYTHON_BINDINGS:BOOL=OFF
```

Compile and install it:

```
cmake --build build/ --target install
```

It might take a while.

Configure the `MomentumBasedTorqueController`:

```
cd ../paper_sorrentino_ral2024_balancing_torque
cmake -S . -B build/ \
  -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX 
```

Compile and install it:

```
cmake --build build/ --target install
```

## Simulation

Open a terminal and run:

```
conda activate momentumbasedtorquecontrolenv 
export GZ_SIM_RESOURCE_PATH=<user_directory>/paper_sorrentino_ral2024_balancing_torque/config/robots/ergoCubGazeboV1_1/:$GZ_SIM_RESOURCE_PATH
export YARP_ROBOT_NAME=ergoCubGazeboV1_1
```

replacing `<user_directory>` with yours.

Moreover, consider to add this command to your `.bashrc` to avoid running it everytime.

Then, run:

```
yarpserver --write
```

In another terminal, launch `gz-sim`:

```
cd ./paper_sorrentino_ral2024_balancing_torque/config/robots/ergoCubGazeboV1_1/world
YARP_CLOCK=/clock gz sim world.sdf -r
```

In another terminal, run the controller:

```
cd ./paper_sorrentino_ral2024_balancing_torque/config/robots/ergoCubGazeboV1_1
YARP_CLOCK=/clock MomentumBasedTorqueControl
```
