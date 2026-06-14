# QBot 2 (Kobuki) Maze Simulation Environment

Because you need to show a simulation for demonstration **first**, I have built a fully functional 2D Python simulation that implements the **A* Path Planning Algorithm** traversing a maze. This gives you immediate results without waiting hours to install heavy 3D software.

## Phase 1: Run the Immediate Simulation

I have already installed the necessary mathematical and visual libraries on your Windows machine and created the simulation script.

**To run the simulation right now:**
1. Open a Command Prompt or PowerShell in your project folder:
   `c:\Users\User\Desktop\Mecharonics\Sem_7\ME420\Anti_Qbot2`
2. Run the following command:
   ```cmd
   python maze_simulation.py
   ```
3. A graphical window will open showing the grid maze, computing the A* path, and animating the red dot (representing the Kobuki base) moving from the blue Start to the green Goal.

*(You can open `maze_simulation.py` in your code editor to easily change the maze layout by modifying the `maze` array [0 = free space, 1 = wall]).*

---

## Phase 2: Setup the ROS 2 WSL Environment (For the Raspberry Pi)

When you are ready to migrate this algorithm to the real **Raspberry Pi 3 B+ / 4 B+**, you must use ROS 2 inside your WSL environment. 

Since installing ROS 2 downloads gigabytes of data and takes significant time, you should run these commands yourself in your WSL terminal when you have a stable internet connection.

**Step 1: Open your WSL Terminal (Ubuntu 24.04)**
Open your Start menu and launch `Ubuntu`.

**Step 2: Add the ROS 2 Repository**
Run these commands one by one (enter your password `1809baanu` when prompted):
```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

**Step 3: Install ROS 2 Jazzy and Simulation Tools**
This step will take a while to download.
```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz -y
```

**Step 4: Source the Environment**
Add ROS 2 to your bash profile so it loads every time you open WSL:
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Once Phase 2 is complete, we can port the `maze_simulation.py` logic into a ROS 2 Node that publishes `cmd_vel` (velocity commands) to the Raspberry Pi over Wi-Fi!
