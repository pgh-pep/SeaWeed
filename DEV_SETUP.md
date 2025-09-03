# Development Environment Setup

## Installing ROS2

ROS2 and the additional tools we use require Ubuntu 22.04. If you are on Windows, WSL (Windows Subsystem for Linux) is the recommended approach for development. ADD MACOS INFO.

### 1) Install Ubuntu 22.04 w/ WSL2 (If using Windows)

## For Macs, see instructions below.

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
sudo apt install python3-colcon-common-extensions python3-rosdep2 libsdl1.2-dev bash-completion nano python3-pip python-is-python3 -y
echo "source /opt/ros/humble/setup.sh" >> ~/.bashrc
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
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
sudo apt update
sudo apt install gz-garden ros-humble-ros-gzgarden ros-humble-xacro python3-sdformat13
```

Reinstall Gazebo 11 (if desired):

```sh
sudo add-apt-repository ppa:openrobotics/gazebo11-gz-cli
sudo apt update
sudo apt install gazebo11
sudo apt install ros-humble-gazebo-ros-pkgs
```

Once all dependencies are installed, [`CONTRIBUTION.md`](https://github.com/pgh-pep/SeaWeed/blob/main/CONTRIBUTING.md) has build instructions.

## MacOS ROS2 Setup Instructions

### 1. Install Canonical Multipass

[`multipass`](https://canonical.com/multipass/install)


### 2. Setup Ubuntu ROS2 VM with Multipass

In a terminal:

```sh
multipass launch -m 4G -v -n "pep-ros" ros2-humble
  
```

This will create our virtual machine with ROS2 Humble pre-installed. Please ensure you have at least 16 Gigs of RAM before continuing. While 8 Gigs will work, it has caused issues in the past.

Then to get shell access into our VM, type:

```sh
multipass exec pep-ros bash
```
(the third argument being the name we have the VM during creation, you can change this)

Now you will have a bash instance running in your VM! 

### 3. GUI and Rviz

So a shell is great but we want to run GUI apps like Rviz to visualize our nodes and what not.

Install ubuntu desktop:

```sh
sudo apt update
sudo apt install ubuntu-desktop xrdp
```

This will take a bit, since you are installing an entire desktop.

Set a password for our VM (just set it to `pep`)

```sh
sudo passwd ubuntu
```
Now exit the session by typing in `exit` and run the following:

```sh
multipass info pep-ros
```

You can get a list of `multipass` commands by running `multipass --help`

You will get a list of information about the VM, copy down the `ipv4` address. You will need this to connect to the VM.

Install Windows Remote Desktop app from the Mac store (trust me, I know it's weird)

Open the app and click on `Add a new PC`

Paste the address you copied earlier and give it a friendly name (like pep-ros, though it doesn't have to be the same name as before)

Enter your credentials (default username: `ubuntu`) and password you made earlier.

To open Rviz, open up a terminal in the Ubuntu VM and run:

```sh
ros2 run rviz2 rviz2
```

This should launch Rviz GUI. Now you're all good to go. See CONTRIBUTING.MD to get your local version of SeaWeed setup on the VM. Checkout the setup git steps above
to get the repository pulled on the VM.

### 4. Development on Macs

The GUI desktop is not great, so would recommend doing all development using the `multipass exec pep-ros bash` command and saving the GUI for when you need to run Rviz
or any other GUI ROS apps.
