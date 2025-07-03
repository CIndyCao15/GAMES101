This project implements a simple 3D triangle rasterizer. It demonstrates the use of transformation matrices, including rotation around the Z-axis and arbitrary axes using Rodrigues’ rotation formula, along with a perspective projection matrix.


## Features

- Rotation using a **model matrix** around the Z-axis
- Construction of a **perspective projection matrix** with adjustable FOV, aspect ratio, near and far planes
- **Rodrigues rotation matrix** for arbitrary-axis rotation
- **Interactive controls** for rotating and switching rotation modes


## Controls

| Key | Action |
|-----|--------|
| `A` | Rotate clockwise |
| `D` | Rotate counter-clockwise |
| `M` | Toggle between Z-axis rotation and Rodrigues rotation |
| `Esc` | Exit the program |


## Implementation Notes

- The **Z-axis model matrix** rotates the triangle around the origin on the XY plane.
- The **Rodrigues rotation matrix** enables rotation around any axis passing through the origin (e.g. `{0, 1, 0}`).
- In the **projection matrix**, I initially used `atan()` instead of `tan()`, which caused incorrect projection scaling.
- The output image appeared **upside-down** because OpenCV’s coordinate system starts from the **top-left** corner. I fixed this by adding a vertical flip using `cv::flip`.