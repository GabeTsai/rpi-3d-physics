# rpi-3d-physics

Contributors: Rohan Bhowmik, Gabriel Tsai

Final project for **Stanford's CS140E - Embedded Operating Systems**: A bare-metal 3D graphics and physics engine on the RPi Zero's VideoCore IV GPU. 

### Overview

We built a basic 3D graphics and physics engine on the RPi using the VideoCoreIV’s 3D pipeline that supports the rendering of various platonic solids in fixed lighting with flat or Gouraud shading, a PS2 keyboard-controlled camera with accurate perspective, rendering of blender .obj files, collision detection and gravity fields. 

The core of our pipeline centers around the V3D’s control lists - sequences of variable length control records which each contain a single byte id code followed by a specific number of bytes of data specifying information on primitives (points, lines, triangles, etc.), settings in the graphics pipeline, or branches or links to other control lists. We built the VideoCoreIV 3D pipeline as specified by the datasheet - a tile-based deferred renderer with two stages:

- **Binning**: primitives are assigned to screen tiles.
- **Rendering**: tiles are processed in parallel, with QPUs executing fragment shaders (QPU programs written in QASM that color pixels generated during rasterization).

QPU scheduling and tile work distribution are handled automatically by the hardware. Once control lists are constructed for binning and rendering, the host writes the addresses of those control lists to specific V3D registers to execute them and the 3D pipeline will then automatically bin and render all specified primitives to the framebuffer. 

The V3D provides multiple programmable pipeline modes - we use the Non-Vertex Shading (NV) mode, which required us to write our own CPU-side programs called vertex shaders to calculate vertex positions, convert them into triangles and format them into primitive lists to be ingested into the V3D pipeline. 

Our engine constructs meshes from triangle primitives, and supports the creation of triangles, planes, boxes, icosa-, tetra-, octa-, and dodecahedrons, as well as icospheres with n subdivisions. We use a basic lighting system where all light occurs from one direction in a “sun” like fashion, and support flat shading, where each triangle is rendered the same color, or Gouraud shading, where vertex colors are interpolated across the edge boundaries of faces to create a smooth look. 

In order to render objects as they would appear through a camera, we make use of the pinhole camera model in which the visible portion of 3D space is composed of a rectangular frustum (truncated pyramid), its tip at the camera position and base representing screen view. Projecting the pyramid space into an orthogonal space allows us to turn points, triangles, and thus geometry in 3D space into camera space.

We can then attribute physical properties of position and orientation + their first derivatives (linear/angular velocity) + second derivatives (linear/angular acceleration). We assign masses and consult mesh geometry to calculate moment of inertia, thus allowing us to apply forces and torques to mediate linear/angular accelerations. Running simulations involve incrementing time steps and calculating objects’ updated positions and orientations according to second-order kinematics equations.

Finally to simulate collisions, we use the Gilbert–Johnson–Keerthi distance (GJK) algorithm to determine if two objects’ meshes overlap (collide) and the Expanding Polytope Algorithm (EPA) to further calculate the closest points on each object to the collision center, allowing use to determine the collision direction and depth. This allows us to apply a post-physics correction step to push colliding objects apart and determine collision response (conservation of linear and angular momentum).
