# Structure-from-Sherds (Pre-processing)

This repository contains the **pre-processing** code for the Structure-from-Sherds project, which focuses on preparing pottery fragment data for reconstruction. The pre-processing pipeline includes:

1. **Mesh-to-Surface**  
2. **Breakline Extraction**


## Overview
Pottery fragments (3D scans in `.obj` format) are turned into processed point clouds with relevant surface information and geometric features that aid in the reconstruction process.

## Dependencies
- PCL (Point Cloud Library): 1.9.1
  - Required for point cloud processing and surface analysis
- CGAL: 5.0
  - Used for mesh processing and surface operations
- C++ 17 or higher

## Building the Project

### Using Docker

1. Build the Docker image:
   ```bash
   docker build -t sfs_pre:latest .
   ```
2. Download Dataset
   ```
   sh ./download.sh
   ```
3. Run docker container
   ```
   sh ./setup_container.sh
   ```
   - You might have to change --volume="/Dataset:/Dataset" \ at line 20 according to your dataset path
4. Build with cmake file
   ```
    ## Command at container ##
    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release 

    make
   ```
5. Run the program

> **Notice**: The code for axis estimation is currently being refactored and will be released soon. Stay tuned for updates!

## Pre-processing Pipeline (Mesh2Surface)

### 1. Mesh to Point Cloud Conversion
- Input: Raw 3D mesh files (.obj format)
- Process: 
  - Converts mesh to point cloud using CloudCompare
  - Samples up to 1,000,000 points per mesh
  - Initial noise filtering
- Output: PCD format point cloud

### 2. Surface Processing
- Normal Estimation and Orientation
  - Computes surface normals for each point
  - Automatically corrects normal vector orientations
  - Uses PCA-based normal estimation with k-nearest neighbors

- Uniform Sampling
  - Reduces point cloud density while maintaining surface features
  - Removes noise and outliers
  - Configurable sampling radius

- Surface Segmentation
  - Region growing-based segmentation
  - Separates different surface regions based on:
    - Normal vector similarity
    - Surface curvature
    - Spatial proximity

### 3. Surface Analysis and Refinement
- B-Spline Surface Fitting
  - Improves surface boundary definition
  - Reduces noise in surface representation
  - Maintains geometric features

- Cluster Analysis
  - Identifies and separates distinct surface regions
  - Merges related segments based on geometric criteria
  - Handles special cases (decorative parts, damage)

## Pre-processing Pipeline (Surface2Breakline)

After finishing **Mesh2Surface** steps (generating `*_Surface_X.xyz`), we run the **breakline** code to extract and refine boundaries on each fragment. This process produces a final boundary file, `*_CompleteBreaklines.xyz` or `.pcd`, which can be used for matching or assembly in downstream tasks.

1. **Breakline Detection**  
   - **Initial boundary detection** based on surface normals (e.g., `BoundaryEstimation` in PCL)  
   - Automatic ordering of boundary points into continuous segments

2. **Smoothing & Resampling**  
   - Each breakline is **resampled using B-spline** fitting to remove noise  
   - Sharp curvature regions can be split further using **peak detection**

3. **Optional Projection & Classification**  
   - Fit a global B-spline surface to the entire fragment; project breakline points for refined normals  
   - Classify breaklines (e.g., **rim** vs. **non-rim**) if pot axis data is available  

4. **Output**  
   - **Final breaklines** (`*_CompleteBreaklines.xyz` or `.pcd`) – **primary** output 

## Configuration Parameters

### Surface Segmentation Parameters
- `minCluster`: Minimum cluster size (default: 50)
- `noOfNeighbours`: Number of neighbors for region growing (default: 10)
- `smoothnessAngleThreshold`: Angle threshold for smoothness (default: 4.0)
- `curvatureThreshold`: Threshold for curvature calculation (default: 1.0)

### Processing Parameters
- `samplingRadius`: Radius for uniform sampling
- `normalEstimationNeighbors`: Number of neighbors for normal estimation
- `mergingThreshold`: Threshold for cluster merging

### Breakline Extraction Parameters
- `breaklineSearchRadius`: Neighborhood radius around breakline points (default: 3.0)
- `bSplineOrder`: Polynomial order for B-spline smoothing on breaklines (commonly 3)


### Output Files
- **Point Cloud Files**  
  - `*_Point.pcd`: Initial point cloud conversion  
  - `*_SampledWithNormals.ply`: Point cloud with computed normals  
  - `*_Surface_0.ply/xyz`, `*_Surface_1.ply/xyz`: Segmented surfaces  

- **Breakline Files**  
  - `*_CompleteBreaklines.xyz` or `.pcd`: **Refined final boundary curves** (primary output of Surface2Breakline)


## License
[Add your license information here]

## Citation
If you use this code in your research, please cite:
```
@inproceedings{YooandLiu2024SfS,
	author    = {Yoo, Seong Jong and Liu, Sisung and Arshad, Muhammad Zeeshan and Kim, Jinhyeok and Kim, Young Min and Aloimonos, Yiannis and Fermüller, Cornelia and Joo, Kyungdon and Kim, Jinwook and Hong, Je Hyeong},
	title     = {Structure-From-Sherds++: Robust Incremental 3D Reassembly of Axially Symmetric Pots from Unordered and Mixed Fragment Collections},
	year      = {2024},
}
```
```
@inproceedings{HongandYoo_2021_ICCV,
	author    = {Hong, Je Hyeong and Yoo, Seong Jong and Zeeshan, Muhammad Arshad and Kim, Young Min and Kim, Jinwook},
	title     = {Structure-From-Sherds: Incremental 3D Reassembly of Axially Symmetric Pots From Unordered and Mixed Fragment Collections},
	journal   = {Proceedings of the IEEE/CVF International Conference on Computer Vision (ICCV)},
	month     = {October},
	year      = {2021},
	pages     = {5443-5451}
}
```