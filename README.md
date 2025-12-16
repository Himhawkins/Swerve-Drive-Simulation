# 🤖 Swerve Drive Physics & Path Planning Simulation

A comprehensive Python simulation suite for Holonomic (Swerve Drive) robots. This project ranges from basic kinematic movement to complex dynamic physics (inertia/friction), procedural maze generation, and potential field path planning with dynamic obstacle avoidance.

## ✨ Features

* **Swerve Kinematics:** Math for independent wheel steering and driving.
* **Physics Engine:** Simulates mass, moment of inertia, linear/angular drag, and wheel friction (static vs. kinetic).
* **Path Planning:** Uses Potential Fields and Cost Maps to navigate complex environments.
* **Procedural Generation:** Randomly generates mazes using depth-first search algorithms.
* **Pure Pursuit Control:** Path tracking algorithms to follow generated routes.
* **Dynamic Obstacle Avoidance:** Real-time reaction to new obstacles added during simulation.

## 🛠️ Installation

### Prerequisites
* Python 3.8 or higher

### Install Dependencies
This project relies on `pygame` for visualization and scientific libraries for math and pathfinding.

```bash
pip install pygame numpy scipy scikit-image
