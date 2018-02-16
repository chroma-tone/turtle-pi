To create the turtle_pi_chassis_collision.dae file:

1. Export the assembly from Onshape - Choose an STL output, and uncheck "Export unique parts as individual files"
2. Import the STL into Blender - File -> Import -> .stl
3. Select the imported object, and add a "Decimate" modifier. Wind down the Ratio until the model doesn't explode (I used 0.15)
4. Type "n" to open the Properties toolbar and Scale the whole object by 0.001 to get to mm.
5. Click Object -> Apply -> Scale to apply this scale.
6. File -> Export -> STL, and save as turtle_pi_chassis_collision.stl
7. Import the file into Meshlab, then re-export as .dae

To calculate the Inertia Tensor:
1. Open the file in Meshlab
2. Filters -> Quality Measures and Computations -> Compute Geometric Measures
3. The Inertia Tensor will be displayed in the output window.
4. However, the values need to be divided by the volume to get the final answer
5. These results can be copy / pasted in to the urdf file.