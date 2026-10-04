# Path Planning Task

## Overview

This project implements a simple path-planning algorithm for a car driving between blue and yellow cones.

- Blue cones (`color == 1`) represent the **left** side of the track.
- Yellow cones (`color == 0`) represent the **right** side of the track.
- The car pose is given by `(x, y, yaw)`.
- The generated path consists of `(x, y)` points in world coordinates.

The main implementation is in:

```text
src/path_planning.py

Part 1
The path planner handles three main cases.
1. No cones
When no cones are visible, the car follows a straight path using its current heading.
2. Both blue and yellow cones are visible
When cones from both sides are available, blue cones are matched with the closest yellow cones.
For every matched pair, the midpoint is calculated:
center_x = (blue_x + yellow_x) / 2
center_y = (blue_y + yellow_y) / 2

The midpoint represents the estimated center of the track.
The path starts from the car position and initially follows the car's heading. It then smoothly moves toward the estimated centerline.
When a sharp change in direction is detected, a cubic Bezier curve is used to smooth the turn instead of creating a sharp corner.
3. Only one side of the track is visible
If only blue or only yellow cones are available, the other side of the track cannot be directly observed.
I assume a track width of approximately 4 m, so the center of the track is estimated 2 m from the visible boundary.
- Blue cones are on the left, so the center is estimated 2 m to the right.
- Yellow cones are on the right, so the center is estimated 2 m to the left.
When multiple cones are available on one side, each cone is used to estimate a corresponding centerline point.


Part 2
Three cones on one side
For Part 2, the algorithm was extended to use three cones on the same side of the track.
Instead of using only one cone, the planner processes all visible cones and estimates a centerline point for each one.
For example, with three blue cones:
Blue        Blue        Blue
  ●           ●           ●
  |           |           |
  |           |           |
  ×           ×           ×
       estimated centerline

Since blue represents the left boundary, the estimated centerline is placed approximately 2 m to the right of each blue cone.
The same idea is used for three yellow cones, except the centerline is estimated 2 m to the left of the yellow boundary.
Why this approach?
The approach was chosen because it is simple and does not require a complex path-planning algorithm.
Using three cones gives more information about the shape of the visible boundary than using only one cone. This allows the planner to create several centerline points and follow the estimated track direction.

Assumptions
The following assumptions are made:
- Blue cones are always on the left side of the track.
- Yellow cones are always on the right side of the track.
- The approximate track width is 4 m.
- Therefore, when only one boundary is visible, the track center is estimated 2 m from that boundary.
- The available cones represent the currently relevant section of the track.
- The path is generated in world coordinates.

Limitations
This is a simple geometric approach, so it has some limitations.
- When only one side of the track is visible, the exact track center cannot be known because the opposite boundary is unknown.
- The 4 m track-width assumption may not be correct for every track.
- A complicated track with sharp turns may require more advanced path planning.
- Cones that belong to different sections of a complicated track could lead to an incorrect estimated centerline.
- The algorithm does not use vehicle dynamics, steering limits, or obstacle avoidance beyond the available cone information.

Testing
The original scenarios from 1 to 20 were tested.
Additional scenarios were added for Part 2 to test three cones on one side:
- Scenario 21: three blue cones on a straight boundary.
- Scenario 22: three yellow cones on a straight boundary.
- Scenario 23: three blue cones forming a diagonal boundary.
- Scenario 24: three blue cones forming a rising boundary.
- Scenario 25: three yellow cones forming a rising boundary.
- Scenario 26: three yellow cones forming a diagonal boundary.
- Scenario 27: three blue cones on a horizontal boundary.

The scenarios can be run using:
python -m src.run --scenario N

For example:
python -m src.run --scenario 21

Project Structure
.
├── src/
│   ├── models.py
│   ├── path_planning.py
│   ├── scenarios.py
│   ├── run.py
│   └── tester.py
├── requirements.txt
└── README.md

Running the Project
Create a virtual environment if desired:
python -m venv .venv

On Linux/macOS:
source .venv/bin/activate

On Windows:
.venv\Scripts\activate

Install the dependencies:
pip install -r requirements.txt

Run a scenario:
python -m src.run --scenario 1

Replace 1 with any available scenario number.
```