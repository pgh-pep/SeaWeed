# Development Environment Setup

## Installing ROS2

ROS2 and the additional tools we use require Ubuntu 22.04. If you are on Windows, WSL (Windows Subsystem for Linux) is the recommended approach for development. ADD MACOS INFO.

### 1) Install Ubuntu 22.04 w/ WSL2 (If using Windows)

Make sure Windows is up to date.

NOTE: WSL install instructions vary depending on Windows version. For help on any errors that may occur:

- <https://learn.microsoft.com/en-us/windows/wsl/install-manual>
- <https://learn.microsoft.com/en-us/windows/wsl/troubleshooting>

To install WSL2, in a PowerShell or Windows Command Prompt, run:

```powershell
wsl --install -d Ubuntu-22.04
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

Add the ROS 2 apt repository to your system:
```sh
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

Update and upgrade apt repository again:
```sh
sudo apt update && sudo apt-get upgrade -y
```

Install ROS2 Humble:
```sh
sudo apt install ros-humble-desktop -y
```

ROS must be sourced every time the terminal is launched. Install additional dependencies and, in the `~/.bashrc` file, automatically source ROS and colcon autocompletion.

```sh
sudo apt install python3-colcon-common-extensions python3-rosdep2 libsdl1.2-dev sh-completion nano python3-pip python-is-python3 -y
echo "source /opt/ros/humble/setup.sh" >> ~/.bashrc
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.sh" >> ~/.bashrc
source ~/.bashrc
sudo rosdep init
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


## Installing Gazebo Garden

For additional information regarding the Gazebo versions, URDFs, bridges & related topics, visit the [PEP Docs](https://github.com/pgh-pep/pep_resources/blob/main/Simulation/gazebo.md).


If you already have ROS2 Humble installed, you might already have Gazebo 11 and Gazebo/Ignition Fortress installed. We need to remove conflicting Gazebo versions first to prevent library conflicts:

```sh
sudo apt remove ign*
sudo apt-get remove gazebo*
```

Install necessary dependencies:

```sh
sudo apt-get update
sudo apt-get install lsb-release curl gnupg
```

Install Gazebo Garden:

```sh
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-garden ros-humble-ros-gzgarden ros-humble-xacro python3-sdformat13
```

Reinstall Gazebo 11 (if desired):

```sh
sudo add-apt-repository ppa:openrobotics/gazebo11-gz-cli
sudo apt update
sudo apt-get install gazebo11
sudo apt install ros-humble-gazebo-ros-pkgs
```

Once all dependencies are installed, [`CONTRIBUTION.md`](https://github.com/pgh-pep/SeaWeed/blob/main/CONTRIBUTING.md) has build instructions.