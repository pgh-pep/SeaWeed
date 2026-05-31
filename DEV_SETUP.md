# Development Environment Setup

## Installing ROS2 Jazzy

ROS2 and the additional tools we use require Ubuntu 24.04. If you are on Windows, WSL (Windows Subsystem for Linux) is the recommended approach for development. ADD MACOS INFO.

### 1) Install Ubuntu 24.04 w/ WSL2 (If using Windows)

Make sure Windows is up to date.

NOTE: WSL install instructions vary depending on Windows version. For help on any errors that may occur:

- <https://learn.microsoft.com/en-us/windows/wsl/install-manual>
- <https://learn.microsoft.com/en-us/windows/wsl/troubleshooting>

To install WSL2, in a PowerShell or Windows Command Prompt, run:

```powershell
wsl --install -d Ubuntu-24.04
# If getting errors, try in another CMD:  wsl --set-default-version 2
```

You will be prompted to enter a username and password, followed by a successful installation message.
(Your password will not be shown when typing for security reasons)

To confirm a successful installation of WSL and Ubuntu, you can list your currently installed distros with:

```powershell
wsl --list -v
```

### 1.5) Using WSL2 w/ VSCode (You can come back to this after everything is installed)

Open up [VSCode](https://code.visualstudio.com/download) on your Windows machine. Install the Remote Development Extension.
In the bottom-left, click on the Remote Window button (blue w/ arrows).
![alt text](https://canonical-ubuntu-wsl.readthedocs-hosted.com/en/latest/_images/remote-extension.png)

Select `Connect to WSL using Distro` and select `Ubuntu-22.04`.

This will open VSCode in your newly made WSL environment.

### 2) ROS Installation

Open the bash terminal of your newly installed distro by opening the Ubuntu app.

Update and upgrade packages:

```sh
sudo apt update && sudo apt upgrade -y
```

Set locale:
```sh
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

Add the ROS 2 apt repository to your system:

```sh
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

Update and upgrade apt repository again:

```sh
sudo apt update && sudo apt-get upgrade -y
```

Install ROS2 Jazzy:

```sh
sudo apt install ros-jazzy-desktop
```

ROS must be sourced every time the terminal is launched. Install additional dependencies and, in the `~/.bashrc` file, automatically source ROS and colcon autocompletion.

```sh
sudo apt install python3-colcon-common-extensions python3-rosdep libsdl1.2-dev bash-completion nano python3-pip python-is-python3 -y
echo "source /opt/ros/jazzy/setup.sh" >> ~/.bashrc
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
source ~/.bashrc
sudo rosdep init
rosdep update
```

## Installing Gazebo Harmonic

For additional information regarding the Gazebo versions, URDFs, bridges & related topics, visit the [PEP Docs](https://github.com/pgh-pep/pep_resources/blob/main/Simulation/gazebo.md).

Install Gazebo Harmonic:
```sh
sudo apt install ros-jazzy-ros-gz
```

## Git Setup

Set up Git credentials that match your GitHub information:

```sh
git config --global user.name "your_user_name"
git config --global user.email "youremail@domain.com"
```

Connect with a [SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent):
[SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent):

```sh
# Generate SSH key (Skip ALL prompts w/ enter):
ssh-keygen -t ed25519 -C "your_email@example.com"
# Display public key in terminal:
cat ~/.ssh/id_ed25519.pub
# Copy the entire key
```

Add the key to your [github account](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account): `Settings > SSH and GPG Keys > New SSH Key`


Once all dependencies are installed, [`CONTRIBUTION.md`](https://github.com/pgh-pep/SeaWeed/blob/main/CONTRIBUTING.md) has build instructions.
