## Terminology
- **Primitive Setup Engine (PSE)**, responsible for setting up data for rasterizing primitives and coefficients of equations for interpolating varyings
- **Primitive Tile Binner (PTB)** - fetches transformed vertex coords from VPM and works out overlaps any possible overlaps between tiles

## Tile Buffer - Section 5

The Tile Buffer, or TLB actually consists of 3 buffers: 
- **color**
- **Z**
- **stencil** 
- **alphamask** 

QPUs can directly read/write to color/alphamask buffers. Access to Z/stencil buffer is more specialized

## Control Lists - Section 9
**control lists** are the bread and butter of essentially all operations of the VideoCoreIV's 3D system. Once the CLE has started executing a control list, the host barely needs to do anything. 

**Control lists** are sequences of variable length **control records** - "each record starts with a single byte id code followed by an implicit number of bytes of data" (65). Each control record specifies either a:
- Primitive index list
- A piece of state data
- A system control action
- A branch or link to another control list

## System Control - Section 8
The control list executor (CLE) manages reading of control lists, controls, feeds the 3D pipeline

The CLE has two separate threads used to execute the required separate **binning** and **rendering** passes to render a frame. 
**Thread 0** is dedicated to tile binning, **Thread 1** is dedicated to tile rendering. 

Thus, two control lists - one for the binning pass, one for the rendering pass are required. 

The **binning list** sets up the tile binning mode config, supplies binning memory for the **PTB** to work with, specifies all state data, shaders and primitive lists to complete frame. 

**Primitive lists** are sequences of vertices interpreted as primitives (triangles, lines, etc) according to whatever primitive format is specified in a control record. 

There are two separate passes - the binning and the rendering pass
#### Binning Pass
- PTB writes new control list for rendering ecah tile during the rendering pass
- Must finish with **flush** command to cause PTB to finalize all these tile lists
#### Rendering Pass
- For each tile list, the host should attach a 'tile coords' item before and a 'store tile' item after each tile list 
- Terminate with a 'store tile plus end of frame' item after last tile list

**The host can write the binning and rendering list to the appropriate registers before the binning pass has even started**

To supply the control list to thread n, specify the start/end list addresses to the **V3DCTnCA** and **V3DCTnEA** registers. The host can also extend the list at any time by updating **V3DCTnEA**, causing processing to resume if the control thread has already reached the end of the list. 

### Pipelines

There are 3 different pipelines the VideoCoreIV uses for tile-binning/rendering - GL, NV, and VG. GL mode means you'll use the fixed-function OpenGL style pipeline since it will use OpenGL's vertex shader drivers, which isn't really in the spirit of bare metal because we'd have to pull in OpenGL drivers so we'll use **NV** mode, which requires you to provide your own vertex shader. It is also significantly more gangster to write your own vertex drivers. 

#### Aside - Vertex Shaders

The vertex shader takes vertex attributes supplied in memory, does a few math ops and then stores the shaded vertex into the shaded format the GPU rasterizer can use. 

#### NV Pipeline

1. Determine batch of vertices to shade in the VCM (Vertex Cache Manager)
2. Find space in VPM to store vertex input attributes and shaded vertices
3. Fetch vertex attributes to VPM using VCD (Vertex Cache DMA)
4. Load in pre-shaded vertex data from memory to VPM (or pre-compute them with a QPU kernel)
5. PTB/PSE reads shaded vertex data from VPM

## Dev Log
#### Aside on the NV clip header no point size shaded vertex format

**3/17/26, 4:08 AM** - The hardware clipping path for NV pre-shaded vertices is very finnicky and we decided to prioritize getting a stable end-to-end renderer and physics engine. There are no working public references that use this particular NV shaded vertex format. 

Despite theoretical minimal modification to the simple triangle render test to that would enable hardware clipping, we spent around 24 hours combined dev time trying to get it to work. After at least 12 - 15 different fixes (while maintaining the as small code deltas as possible), we proved that the tile binning phase was working correctly - a dump of a heap allocated array that the PTB's internal logic fills with primitive data before and after execution of the binning control list showed the array went from all zeros to containing valid primitive data. Yet, the rendering pass completely failed. Hardware clipping is most useful if vertices end up behind the camera (W_c is negative) - since we divide clip space coordinates by W_c, this inverts position, resulting in inside out triangles that fill the screen with garbage. We should be able to mimick the hardware clipping functionality with a CPU function - it will cost a bit more compute but the check should be O(n) for n triangles.
-Gabe  

**3/19/26, 8:46 AM** - I have slept 30 minutes in the last 24 hours. We encountered a really bad bug that was exposed via a simple collision test - a cube falling onto a large flat cube (plane). Every time the cube collided with the plane, the program would freeze. Turns out the cause was the collision logic outputting floats that were way below the normal float threshold (de-norms), causing underflow. We tried to manually correct all of the multiplications but this just wasn't tractable. Page 1-20 of the ARM1176JZF-S Technical Reference Manual states that the VFP hardware will raise an undefined instruction exception for all 5 IEEE 754 floating point exceptions including underflow so for bare metal, the user should implement logic to catch that and compute the correct IEEE 754 result in software. For a physics engine, the difference between the smallest IEEE 754 normal (magnitude on 10e-38) and 0 is negligible so the cleanest solution is to enable **RunFast Mode**, which tells the VFP to auto flush anything that underflows to zero and keep going. Moral of the story is to always consult the datasheet when you can and see if you can take advantage of existing hardware. No matter how obscure the problem seems, some caffeinated engineer probably encountered it before you and wrote a fix for it. 
