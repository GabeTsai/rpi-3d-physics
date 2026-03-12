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