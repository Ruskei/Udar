# Udar

Udar is a real-time 3D rigidbody physics engine with a Minecraft frontend for easy visualization and testing. It simulates cuboidal and convex composite rigidbodies and supports contact, position, hinge, and cone constraints.

<img width="2964" height="1804" alt="top_view" src="https://github.com/user-attachments/assets/1dbbe63c-8760-4869-800f-9ad385f1c53f" />

<img width="2964" height="1804" alt="crash" src="https://github.com/user-attachments/assets/a85d6c61-c83c-44ca-8958-50bee01f48bc" />

(Views of 2000 cuboid pile being simulated at 60Hz)

### Features
- Full 3D rigidbody dynamics (position, velocity, quaternion orientation, angular velocity)
- Eigenvector decomposition for finding inertia tensor of composites
- RK4 integration
- Parallel broadphase and narrowphase collision detection
- Minecraft world physics mesh generation and collision
- Sequential impulse (PGS) constraint solving
- Friction solving
- Pontact, position, hinge, cone constraints
- Runtime visualization
- Timing telemetry

### Simulation breakdown

Every simulation tick is split into several subticks, following the steps:

1. Process external physics queries (object addition / removal)
2. Update sleep state of bodies
3. Broadphase: find potential collisions with dynamic AABB tree
4. Narrowphase: confirm collisions and generate contact manifolds
5. Perform collisions with static geometry
6. Apply gravity
7. Solve velocity constraints
8. Integrate body position and orientation
9. Solve position constraints
10. Store contact data for warm starting

### Problems solved & learnings

- Developed custom collision detection algorithm based on decomposition into face and edge collisions, then using a graph structure to create a proper contact manifold
- Researched and tested various constraint solving and contact manifold generation algorithms to reduce energy errors and jitter
- Practiced and applid multivariable calculus and linear algebra to rederivation of constraint solving and collision detection formulas used in this project
- Quaternion math and how they encode orientation, as well as their relationship to axis angle representation and how they solve the gimbal lock problem of Euler angles
- Writing fast multithreaded code for the JVM with minimal memory allocations

### Building

Download source and ensure you have Java 21 available on your system, and run it on a Minecraft 1.21.4 Paper server. Use the shadowJar gradle build task to build the project.

### Story of this project

I began this project after my AP Physics 1 and AP Calc BC class, inspired to write a full-fledged physics engine. I started by reading the original 1988 _A Fast Procedure For Computing the Distance Between Complex Objects in Three-Dimensional Space_ by Gilbert, Johnson, and Keerthi, and then implementing contact manifold generation off of that (EPA).

However this algorithm is optimized for complex shapes and those required more complicated inertia tensor calculations; I ended up using cuboids, but I wanted to implement a more novel approach to collision detection, so I turned to SDFs, or signed distance fields. I read _Real-time Collision Detection between General SDFs_ by Liu et al. and worked on optimizing their algorithm to my simplistic usecase, but eventually reached the conclusion that these approaches were hindering my project.

Thus I finally switched to SAT (Separating Axis Test) collision detection. I then came up with a completely novel collision detection algorithm for Minecraft's concave voxel geometry based on face and edge primitives combined with a graph structure. This is still used in the current implementation and solves some of the issues that arise with traditional convex decomposition.

I also tried an experimental mass-splitting approach to constraint solving based on _Mass Splitting for Jitter-Free Parallel Rigid Body Simulation_ by Tonge et al. but the poor convergence proved to not be viable.

Now the project has converged to a standard PGS solver, as turned out to be industry standard for a reason, combined with some of Jolt's techniques like a post-velocity position solve step.
