# SeaWeed

## Recommended File Structure
```
~/pep
└───seaweed_ws   (colcon build HERE)
│   └───build    (auto-generated w/ colcon)
│   └───install  (auto-generated w/ colcon)
│   └───log      (auto-generated w/ colcon)
│   └───src
│       └───SeaWeed (Clone HERE)
└───vrx_ws       (colcon build HERE)
│   └───build    (auto-generated w/ colcon)
│   └───install  (auto-generated w/ colcon)
│   └───log      (auto-generated w/ colcon)
│   └───src
│       └───vrx    (Clone HERE)

```

## Installation
add scripts to install seaweed, building, and sourcing

## Simulation

# Install Gazebo Garden
TBD

# Install the VRX simulator:
```sh
mkdir -p ~/pep/vrx_ws/src
cd ~/pep/vrx_ws/src
git clone git@github.com:pgh-pep/vrx.git --branch humble
cd ~/pep/vrx_ws
colcon build --merge-install

# Can manually source in terminal or add to ~/.bashrc to be sourced automatically
source ~/pep/vrx_ws/install/setup.bash
```
