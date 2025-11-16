# 🚁 Drone ArUco Landing System

Autonomous drone landing system on moving platform using ArUco marker detection and PID controller.

## 📋 Description

This project enables a drone to detect and automatically land on a moving platform equipped with an ArUco marker. The system uses an adaptive PID controller to track the moving target, even when visual detection is lost at low altitude.

### Key Features

- ✅ Real-time ArUco marker detection via onboard camera
- ✅ Multi-axis PID controller with altitude-adaptive gains
- ✅ Trajectory memory to maintain tracking below 1.7m
- ✅ Integration with PX4 Autopilot and Gazebo Simulator
- ✅ Offboard mode support for precise control
- ✅ Two versions: simple proportional control and complete PID

## 🛠️ Technologies Used

- **Language**: C++17
- **Autopilot**: PX4 Autopilot
- **Simulator**: Gazebo (gz-sim)
- **Vision**: OpenCV 4.x with ArUco (DICT_4X4_50)
- **Communication**: MAVSDK (MAVLink)
- **Build**: CMake

## 📦 Prerequisites

- Ubuntu 22.04 (ou WSL2)
- PX4-Autopilot installé
- Gazebo Garden ou plus récent
- OpenCV 4.x
- MAVSDK
- CMake 3.10+

## 🚀 Installation

### 1. Clone the repository
```bash
git clone https://github.com/YOUR_USERNAME/drone-aruco-landing.git
cd drone-aruco-landing
```

### 2. Install dependencies

```bash
# OpenCV
sudo apt update
sudo apt install libopencv-dev

# MAVSDK
sudo apt install libmavsdk-dev
```

### 3. Configure the platform model

Copy the `aruco_robot` model to PX4:
```bash
cp -r aruco_robot ~/PX4-Autopilot/Tools/simulation/gz/models/
```

Copy the modified world file to PX4:
```bash
cp gazebo_world/default.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
```

### 4. Compile the project

```bash
cd drone_project
mkdir build
cd build
cmake ..
make
```

## 🎮 Usage

### Launch PX4 + Gazebo simulation

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500_gimbal
```

### Run the control program

In a new terminal:

**Simple version (proportional control):**
```bash
cd ~/drone_project/build
./my_drone_clean
```

**PID version (recommended):**
```bash
cd ~/drone_project/build
./my_drone_pid
```

### Control the moving platform

In another terminal, start the platform movement:
```bash
# Move forward
gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.5}"

# Turn right then move forward
gz topic -t '/cmd_vel' -m gz.msgs.Twist -p 'angular: {z: -1.0}' && sleep 1.57 && gz topic -t '/cmd_vel' -m gz.msgs.Twist -p 'linear: {x: 0.5}'

# Stop
gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0}, angular: {z: 0}"
```

## 📐 Project Architecture

```
drone_project/
├── my_drone_clean.cpp      # Version with simple proportional control
├── my_drone_pid.cpp         # Version with complete PID controller
├── CMakeLists.txt           # Build configuration
├── README.md
├── LICENSE
├── .gitignore
├── gazebo_world/
│   └── default.sdf          # Gazebo world with aruco_robot included
└── aruco_robot/             # Moving platform model
    └── model.sdf            # Gazebo configuration of robot with ArUco
```

## 🔧 System Parameters

### PID Gains (my_drone_pid.cpp)

- **High altitude (> 3m)**: P=2.0, I=0.5, D=0.3
- **Low altitude (< 3m)**: P=1.5, I=0.3, D=0.2

### Speeds

- **Max high altitude**: 2.0 m/s
- **Max low altitude**: 1.5 m/s
- **Descent**: 0.4 m/s (high) → 0.25 m/s (low)

### Altitude Thresholds

- **ArUco detection**: > 1.7m
- **Trajectory memory**: 1.7m - 3.0m
- **Final landing**: < 0.8m

### ArUco Marker Size

- **Recommended**: 0.6m x 0.6m (compromise for far/near detection)
- Adjustable in `aruco_robot/model.sdf`

## 🎯 Operation

### Phase 1: Takeoff (0-10m)
- Automatic arming and takeoff
- Climb to working altitude (10m)
- Gimbal configuration to -90° (camera pointing down)

### Phase 2: Detection and tracking (10m-3m)
- Real-time ArUco detection
- PID control to center the marker
- **Trajectory memorization between 3m and 1.7m**

### Phase 3: Descent with memorized trajectory (1.7m-0.8m)
- Maintain last known trajectory
- No ArUco analysis (too large in field of view)
- Controlled descent at 0.25 m/s

### Phase 4: Final landing (< 0.8m)
- Memorized trajectory reduced by 50%
- Final descent at 0.3 m/s
- Automatic landing at 0.15m

## 📊 Project Impact

### Environmental
- Reduced energy consumption through precise landings
- Prevention of crashes and electronic waste
- Potential applications in sustainable delivery and environmental monitoring

### Social
- Improved safety of drone operations
- Increased accessibility for less experienced operators
- Applications in emergency delivery and rescue

## 🐛 Problèmes connus

- La détection ArUco peut être difficile si le marqueur est trop grand (> 0.8m) ou trop petit (< 0.5m)
- Le robot doit être configuré avec le plugin DiffDrive pour bouger
- Le cache Gazebo doit être nettoyé après modification des modèles : `rm -rf ~/.gz/sim/*`


## 📄 Licence

Ce projet est sous licence MIT - voir le fichier LICENSE pour plus de détails.

## 👤 Auteur

**Brieuc** - Projet de drone autonome avec atterrissage ArUco
