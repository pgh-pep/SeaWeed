### MacOS Instructions w/ Multipass

As an alternative to WSL, we will be using multipass to manage our VMs.

#### 1. Install Canonical Multipass

Install [`multipass`](https://documentation.ubuntu.com/multipass/en/latest/how-to-guides/install-multipass/) using the provided instructions.

#### 2. Setup Ubuntu ROS2 VM with Multipass

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

# 3. GUI and Rviz

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
