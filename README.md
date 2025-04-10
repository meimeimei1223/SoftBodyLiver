# SoftBodyLiver

A real-time simulation of soft liver tissue using Extended Position Based Dynamics (XPBD) to recreate the physical behavior of liver tissue.

[![Watch the demo on YouTube](https://img.youtube.com/vi/UJDAcehueAY/maxresdefault.jpg)](https://youtu.be/UJDAcehueAY "Watch the SoftBodyLiver simulation in action")


## Overview

SoftBodyLiver is a C++ project designed to simulate the physical behavior of soft tissues like the liver in an interactive manner. It can be applied to various fields such as medical training, surgical simulation, and educational purposes.

## Features

- Tetrahedral mesh-based deformation
- Stable simulation using Extended Position Based Dynamics (XPBD)
- Real-time interactive deformation
- Intuitive mouse-based interaction (grab and drag)
- Separation of visual mesh and physics simulation mesh
- 3D rendering with OpenGL

## Technologies

- C++17
- OpenGL / GLEW / GLFW
- GLM (OpenGL Mathematics)
- CMake
- OpenCV (for parallelization)

## Mesh File Formats

The simulation requires two types of mesh files:

1. **Tetrahedral Mesh (.txt)**: Used for physics simulation
   - Format: Custom text file with sections for VERTICES, TETRAHEDRA, EDGES, and SURFACE_TRIANGLES
   - Path: `model/liver_tetrahedral_mesh.txt`
   - This mesh defines the volumetric structure used for physical simulation
   - Contains tetrahedral elements that make up the internal structure of the liver
   - Generated using a modified version of Matthias Mueller's tetrahedralization tool (see Acknowledgements)

2. **Surface Mesh (.obj)**: Used for visualization
   - Format: Standard Wavefront OBJ file format
   - Path: `model/liver.obj`
   - This mesh defines the visual representation of the liver
   - Contains only the surface geometry for rendering purposes

The system uses different meshes for simulation and visualization to balance performance and visual quality. The tetrahedral mesh provides accurate physical behavior, while the surface mesh offers detailed visual representation.

## Building

```bash
mkdir build
cd build
cmake ..
make
```

## Usage

```bash
cd build/bin
./SoftBodyLiver
```

### Controls

- Left-click + drag: Rotate camera
- Mouse wheel: Zoom in/out
- Left-click + drag on the object: Deform the liver
- ESC: Exit
- F1: Toggle wireframe display
- F2: Reset liver shape

## Implementation Details

### Physics Simulation

- XPBD (Extended Position Based Dynamics) for stable soft body simulation
- Tetrahedral elements for volumetric deformation
- Edge constraints to maintain structural integrity
- Volume preservation constraints to simulate incompressible soft tissues
- Spatial hashing for efficient collision detection

### Rendering System

- Vertex skinning to map the physical tetrahedral mesh to the visual surface mesh
- Dynamic normal calculation for realistic lighting
- Support for multiple visualization meshes with transparency
- Option to visualize the tetrahedral structure for debugging

### Performance Optimizations

- Spatial hashing for efficient neighbor finding
- Cached adjacency lists for mesh topology
- Optimized constraint solvers with multiple iterations for stability
- Smart detection and correction of abnormal deformations

## File Structure

- `src/`: Source files
  - `main.cpp`: Application entry point and main loop
  - `SoftBody.h/cpp`: Core simulation classes
  - `VectorMath.h`: Utility functions for vector operations
  - `Hash.h`: Spatial hashing for efficient collision detection
  - `ShaderProgram.h/cpp`: OpenGL shader management
  - `TetoMeshData.h`: Mesh loading and processing utilities

- `shader/`: GLSL shaders
  - `basic.vert`: Vertex shader
  - `basic.frag`: Fragment shader

- `model/`: 3D model files
  - `liver.obj`: Visual surface mesh (Wavefront OBJ format)
  - `liver_tetrahedral_mesh.txt`: Physics tetrahedral mesh (custom format)

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

### Third-Party Libraries

- **GLM (OpenGL Mathematics)**: MIT License
  - [https://github.com/g-truc/glm](https://github.com/g-truc/glm)

### Acknowledgements

This project incorporates and references work from Matthias Mueller:

1. **Soft Body Simulation Code**: The core soft body simulation techniques are based on Matthias Mueller's implementation from [Ten Minute Physics](https://www.youtube.com/channel/UCTG_vrRdKYfrpqCv_WV4eyA).

2. **Tetrahedral Mesh Generation**: The tetrahedral meshes are generated using a modified version of Mueller's Blender plugin for tetrahedralization:
   ```
   Copyright 2022 Matthias Mueller - Ten Minute Physics, 
   https://www.youtube.com/channel/UCTG_vrRdKYfrpqCv_WV4eyA 
   www.matthiasMueller.info/tenMinutePhysics
   ```

Both components are used under the MIT License, which allows for modification and redistribution with proper attribution.

## Future Plans

- Integration with MCUT for mesh cutting operations
- Support for cinolib for advanced mesh processing
- Real-time surgical cutting simulation
- Multi-tissue interaction with varying material properties
- GPU acceleration for improved performance

## Author

Meidai Kasai, MD

## Contributing

Contributions are welcome! Feel free to report bugs, suggest improvements, or submit pull requests.

## Screenshots

[Add screenshots here]
