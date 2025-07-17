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

To develop & run Seaweed (without simulation):
```sh
mkdir -p ~/pep/seaweed_ws/src
cd ~/pep/seaweed_ws/src
git clone git@github.com:pgh-pep/SeaWeed.git
cd ~/pep/seaweed_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
pip install -r requirements.txt
colcon build --symlink-install
```

To add simulation capabilities using the VRX simulator:
```sh
mkdir -p ~/pep/vrx_ws/src
cd ~/pep/vrx_ws/src
git clone git@github.com:pgh-pep/vrx.git --branch humble
cd ~/pep/vrx_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --merge-install
```

You can either manually source the workspaces every time you launch a new terminal or just add to `~/.bashrc` once to be sourced automatically
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
