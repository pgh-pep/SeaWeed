# Contributing

## Recommended File Structure

```sh
~/pep
└───seaweed_ws   (colcon build HERE)
│   └───build    (auto-generated w/ colcon)
│   └───install  (auto-generated w/ colcon)
│   └───log      (auto-generated w/ colcon)
│   └───src
│       └───SeaWeed (Clone HERE)
│       └───external packages (ex. zed_wrapper, ros-drivers, can be added later)
└───vrx_ws       (colcon build HERE)
│   └───build    (auto-generated w/ colcon)
│   └───install  (auto-generated w/ colcon)
│   └───log      (auto-generated w/ colcon)
│   └───src
│       └───vrx    (Clone HERE)
```

### Installing SeaWeed

Clone the SeaWeed repository:
```sh
mkdir -p ~/pep/seaweed_ws/src
cd ~/pep/seaweed_ws/src
git clone git@github.com:pgh-pep/SeaWeed.git
```

Install dependencies:
```sh
cd ~/pep/seaweed_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
sudo apt install ros-humble-joint-state-publisher-gui
cd ~/pep/seaweed_ws/src/SeaWeed
pip install -r requirements.txt
```

Builld w/ colcon (Always build in the workspace directory):
```sh
cd ~/pep/seaweed_ws
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

### Colcon and IntelliSense

For developers new to ROS, colcon is ROS2's build tool that compiles and packages your scripts. Traditionally you would need to rebuild whenever you edit a file. However, by creating symlinks, we only need to rebuild whenever we add/rename/delete a file. Regardless, a lot of issues you will run into can be solved by deleting the old build (`sudo rm -rf ~/pep/seaweed_ws/build ~/pep/seaweed_ws/install ~/pep/seaweed_ws/log`) and rebuilding w/ colcon.

Likewise, with `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON`, we are configuring VSCode's Intellisense to utilize a generated `compile_commands.json` file to locate ROS2 headers. To enable VSCode to use the `compile_commands.json`, add the following to `configurations` of your `.vscode/c_cpp_properties.json`:

```sh
"compileCommands": "~/pep/seaweed_ws/build/compile_commands.json"
```

### Installing VRX Simulator

To add simulation capabilities, clone the VRX simulator:
```sh
mkdir -p ~/pep/vrx_ws/src
cd ~/pep/vrx_ws/src
git clone git@github.com:pgh-pep/vrx.git --branch humble
```

Install VRX dependencies:
```sh
cd ~/pep/vrx_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

Build w/ colcon:
```sh
colcon build --merge-install
```

### Sourcing Workspaces

You can either manually source the workspaces every time you launch a new terminal or source automatically using your `~/.bashrc`:
```sh
echo "source ~/pep/seaweed_ws/install/setup.bash" >> ~/.bashrc
echo "source ~/pep/vrx_ws/install/setup.bash" >> ~/.bashrc
```

## Development Steps

1) Find an issue you want to pick up in the [issue board](https://github.com/orgs/pgh-pep/projects/4) and assign yourself to it (Reach out to Varun for questions)
2) Checkout the branch that you want to base your work off of (Usually this will be `main`):
```sh
git checkout existingBranchName
```
3) Create a new branch that you will develop on
```sh
git checkout -b newBranchName
```
4) Before committing, run linters and formatters and ensure all checks pass (Reach out to Varun for questions)
```sh
pre-commit run --all-files
```
5) To commit changes (If possible, try and follow these [guidelines](https://www.conventionalcommits.org/en/v1.0.0/) for commit messages):
```sh
git commit -m "commit message here"
```
6) To push changes to the git branch you are on:
```sh
git push
```
7) In the [SeaWeed Github](https://github.com/pgh-pep/SeaWeed/pulls), create a pull request (PR) that will be used for code reviews, giving suggestions, and merging your changes into `main`.
