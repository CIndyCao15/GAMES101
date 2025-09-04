This project implements a simple 3D triangle rasterizer. It demonstrates the use of transformation matrices, including rotation around the Z-axis and arbitrary axes using Rodrigues’ rotation formula, along with a perspective projection matrix.


## Features

- Rotation using a **model matrix** around the Z-axis
- Construction of a **perspective projection matrix** with adjustable FOV, aspect ratio, near and far planes
- **Rodrigues rotation matrix** for arbitrary-axis rotation
- **Interactive controls** for rotating and switching rotation modes


## Controls

| Key | Action |
|-----|--------|
| `A` | Rotate counter-clockwise |
| `D` | Rotate clockwise |
| `M` | Toggle between Z-axis rotation and Rodrigues rotation |
| `Esc` | Exit the program |


## Implementation Notes

- The **Z-axis model matrix** rotates the triangle around the origin on the XY plane.
- The **Rodrigues rotation matrix** enables rotation around any axis passing through the origin (e.g. `{0, 1, 0}`).
- In the **projection matrix**, I initially used `atan()` instead of `tan()`, which caused incorrect projection scaling.
- ~~The output image appeared **upside-down** because OpenCV’s coordinate system starts from the **top-left** corner. I fixed this by adding a vertical flip using `cv::flip`.~~
- The output image didn't appear **upside-down**, it appeared **mirrored around the screen center** .We can't flip it because the rotation direction wouldn't be correct. The correct way to do it is to change the input zNear and zFar to negative numbers and correct the projection matrix accordingly.