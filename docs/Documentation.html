<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Swerve Drive Project Documentation</title>
    <style>
        body {
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Helvetica, Arial, sans-serif, "Apple Color Emoji", "Segoe UI Emoji";
            line-height: 1.6;
            color: #24292f;
            max-width: 900px;
            margin: 0 auto;
            padding: 2rem;
            background-color: #ffffff;
        }

        h1, h2, h3 {
            margin-top: 24px;
            margin-bottom: 16px;
            font-weight: 600;
            line-height: 1.25;
        }

        h1 {
            font-size: 2em;
            border-bottom: 1px solid #eaecef;
            padding-bottom: 0.3em;
        }

        h2 {
            font-size: 1.5em;
            border-bottom: 1px solid #eaecef;
            padding-bottom: 0.3em;
            margin-top: 3em;
        }

        h3 {
            font-size: 1.25em;
        }

        p {
            margin-bottom: 16px;
        }

        code {
            font-family: "SFMono-Regular", Consolas, "Liberation Mono", Menlo, Courier, monospace;
            font-size: 85%;
            background-color: #f6f8fa;
            padding: 0.2em 0.4em;
            border-radius: 6px;
        }

        pre {
            background-color: #f6f8fa;
            border-radius: 6px;
            padding: 16px;
            overflow: auto;
            margin-bottom: 16px;
        }

        pre code {
            background-color: transparent;
            padding: 0;
            font-size: 100%;
        }

        ul {
            padding-left: 2em;
            margin-bottom: 16px;
        }

        li {
            margin-bottom: 0.25em;
        }

        hr {
            height: 0.25em;
            padding: 0;
            margin: 24px 0;
            background-color: #e1e4e8;
            border: 0;
        }

        a {
            color: #0366d6;
            text-decoration: none;
        }

        a:hover {
            text-decoration: underline;
        }

        .math {
            font-style: italic;
            font-family: "Times New Roman", Times, serif;
        }
    </style>
</head>
<body>

    <h1>📚 Detailed Project Documentation</h1>

    <p>This document provides a comprehensive technical breakdown of every module in the <strong>Swerve Drive Physics & Path Planning</strong> repository.</p>

    <h2>📋 Table of Contents</h2>
    <ul>
        <li><a href="#path-tracking">1. Main Application: Path Tracking</a></li>
        <li><a href="#physics-engine">2. Physics Engine</a></li>
        <li><a href="#kinematics-engine">3. Kinematics Engine</a></li>
        <li><a href="#path-planner">4. Path Planner</a></li>
        <li><a href="#grid-generator">5. Grid & Maze Generator</a></li>
        <li><a href="#configuration">6. Configuration</a></li>
    </ul>

    <hr>

    <section id="path-tracking">
        <h2>1. Main Application: Path Tracking (<code>path_tracking_lookahead.py</code>)</h2>

        <h3>Description</h3>
        <p>This is the flagship script of the repository. It integrates the Maze Generator, Path Planner, and Physics Robot into a single autonomous loop. The robot navigates from a random <strong>Start</strong> point to a <strong>Goal</strong> point while dynamically avoiding obstacles added by the user.</p>

        <h3>Key Classes</h3>
        <ul>
            <li><strong>HolonomicPurePursuit:</strong> The core controller class.</li>
            <li><strong>Logic:</strong> Calculates the desired velocity vector to reach a "lookahead" point on the path.</li>
            <li><strong>Dynamic Avoidance:</strong> Calculates a secondary repulsion vector based on the gradient of the Cost Map (Heatmap) to push the robot away from sudden obstacles.</li>
        </ul>

        <h3>How to Run</h3>
        <pre><code>python path_tracking_lookahead.py</code></pre>

        <h3>Controls</h3>
        <ul>
            <li><strong>Left Click:</strong> Add a dynamic wall/obstacle at the mouse cursor. The robot will immediately re-plan its path.</li>
            <li><strong>R:</strong> Reset the simulation (New Maze, New Start/Goal, New Path).</li>
        </ul>

        <h3>Technical Flow</h3>
        <ol>
            <li><strong>Init:</strong> Generates Maze &rarr; Calculates Cost Map &rarr; Plans Path.</li>
            <li><strong>Loop:</strong>
                <ul>
                    <li>Robot gets position from Physics Engine.</li>
                    <li>Controller calculates command vector (<span class="math">V<sub>x</sub>, V<sub>y</sub></span>).</li>
                    <li>Physics Engine applies forces to wheels to achieve (<span class="math">V<sub>x</sub>, V<sub>y</sub></span>).</li>
                    <li><strong>If User Clicks:</strong> Adds wall rect to list &rarr; Recalculates Cost Map &rarr; Updates Controller.</li>
                </ul>
            </li>
        </ol>
    </section>

    <section id="physics-engine">
        <h2>2. Physics Engine (<code>d2.py</code>)</h2>

        <h3>Description</h3>
        <p>A high-fidelity simulator that models a robot with mass, inertia, and friction. Unlike simple geometric simulations, this robot will "drift" if it turns too fast and takes time to accelerate.</p>

        <h3>Key Classes</h3>
        <ul>
            <li><strong>DynamicModule:</strong> Represents a single swerve module.
                <ul>
                    <li><strong>PID Control:</strong> Uses a Proportional-Integral-Derivative controller to steer the wheel to the correct angle.</li>
                    <li><strong>Friction:</strong> Simulates static friction (stiction) and kinetic friction.</li>
                </ul>
            </li>
            <li><strong>DynamicRobot:</strong> The chassis object.
                <ul>
                    <li><strong>Integration:</strong> Sums forces from all 4 wheels, applies Drag and Friction, and calculates acceleration (<span class="math">F=ma</span>).</li>
                </ul>
            </li>
        </ul>
    </section>

    <section id="kinematics-engine">
        <h2>3. Kinematics Engine (<code>k2.py</code>)</h2>

        <h3>Description</h3>
        <p>Handles the geometric math required to translate chassis movement into individual wheel movements and vice versa. This module is stateless and purely mathematical.</p>

        <h3>Key Functions</h3>
        <ul>
            <li><code>inverse_kinematics(vx, vy, omega)</code>:
                <ul>
                    <li>Takes desired chassis velocity (<span class="math">V<sub>x</sub>, V<sub>y</sub></span>) and angular velocity (<span class="math">&omega;</span>).</li>
                    <li>Returns the target <strong>Speed</strong> and <strong>Angle</strong> for each of the 4 swerve modules.</li>
                    <li>Normalizes wheel speeds if any command exceeds the physical maximum of the motor.</li>
                </ul>
            </li>
            <li><code>forward_kinematics(module_states)</code>:
                <ul>
                    <li>Takes the current speed and angle of the 4 modules.</li>
                    <li>Returns the estimated chassis velocity (<span class="math">V<sub>x</sub>, V<sub>y</sub>, &omega;</span>).</li>
                    <li>Used for Odometry calculations to estimate robot position on the field.</li>
                </ul>
            </li>
        </ul>
    </section>

    <section id="path-planner">
        <h2>4. Path Planner (<code>planner.py</code>)</h2>

        <h3>Description</h3>
        <p>Responsible for finding the optimal route from Point A to Point B within the grid. It works closely with the Cost Map generated by the grid simulator.</p>

        <h3>Key Algorithms</h3>
        <ul>
            <li><strong>A* (A-Star) Search:</strong> The primary pathfinding algorithm. It uses a heuristic (Euclidean distance) to find the shortest path efficiently.</li>
            <li><strong>Path Smoothing:</strong>
                <ul>
                    <li>Raw A* output produces "jagged" grid-based paths.</li>
                    <li>This module includes a smoothing pass to reduce unnecessary waypoints, creating straight lines where possible.</li>
                </ul>
            </li>
            <li><strong>Node Class:</strong> Represents a specific (<span class="math">x, y</span>) coordinate, storing its <code>g_cost</code> (distance from start), <code>h_cost</code> (distance to end), and parent (for retracing the path).</li>
        </ul>
    </section>

    <section id="grid-generator">
        <h2>5. Grid & Maze Generator (<code>sim_grid.py</code>)</h2>

        <h3>Description</h3>
        <p>Manages the environment data, including the binary obstacle map (Wall vs. Empty) and the gradient cost map (Heatmap) used for safety.</p>

        <h3>Key Features</h3>
        <ul>
            <li><strong>Maze Generation:</strong> Uses a Randomized Depth-First Search (Recursive Backtracker) to generate perfect mazes with no loops.</li>
            <li><strong>Cost Map / Heatmap:</strong>
                <ul>
                    <li>Uses a Distance Transform algorithm.</li>
                    <li>Cells near walls have high "cost" values; cells far from walls have low values.</li>
                    <li>This allows the <code>path_tracking</code> controller to "surf" the gradient valleys, naturally staying away from walls.</li>
                </ul>
            </li>
            <li><strong>Dynamic Updates:</strong> When the user clicks to add a wall, this module updates only the affected regions of the grid to maintain performance.</li>
        </ul>
    </section>

    <section id="configuration">
        <h2>6. Configuration (<code>robot_config.py</code>)</h2>

        <h3>Description</h3>
        <p>A central file for tuning simulation parameters. Changing values here affects physics, kinematics, and graphics instantly.</p>

        <h3>Parameters</h3>
        <ul>
            <li><strong>Physical Constants:</strong>
                <ul>
                    <li><code>ROBOT_MASS</code>: Total mass in kg.</li>
                    <li><code>WHEEL_BASE</code>: Width/Length of the drive base.</li>
                    <li><code>MAX_SPEED</code>: Physical top speed of the motors.</li>
                </ul>
            </li>
            <li><strong>Control Loop:</strong>
                <ul>
                    <li><code>DT</code>: Time step for the physics integrator (e.g., 0.02s).</li>
                    <li><code>PID_GAINS</code>: Tuning values (<span class="math">k<sub>P</sub>, k<sub>I</sub>, k<sub>D</sub></span>) for the module turning motors.</li>
                </ul>
            </li>
            <li><strong>Visualization:</strong>
                <ul>
                    <li><code>SCREEN_WIDTH</code> / <code>SCREEN_HEIGHT</code>: Pygame window dimensions.</li>
                    <li><code>CELL_SIZE</code>: Pixel size of grid blocks.</li>
                </ul>
            </li>
        </ul>
    </section>

</body>
</html>
