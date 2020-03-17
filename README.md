# The Burning Spheres of Dionysus
As it stands, this is a virtual kinetic sculpture designed to work in the Allosphere. It supports live streaming of 3D point clouds created from Kinect data, boids simuated with hashSpace kNN, and a lorentz attractor simulation.
## Dependencies
 * Allollib Playground
 * libfreenect (for the Kinect)
## How to Install
Ensure libfreenect is installed on your machine, and that the allolib_playground repo is cloned. Copy this repo into the Allolib_Playground directory.
## How to Run
CD to allolib_playground, then use the command `./run.sh MAT201B/assignment/distributed/alloEcho-p1.cpp`.
In order to see the distributed state work, open a second terminal and repeat the step above.
**To Enable Kinect Streaming** do `vim MAT201B/assignment/distributed/alloEcho-p1.cpp` and in the `//tweak zone` change the kinectStreaming boolean to `true`. Ensure the kinect is plugged in, then repeat the first two steps.
