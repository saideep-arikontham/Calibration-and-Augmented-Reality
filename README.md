# Calibration-and-Augmented-Reality

## Overview
This project demonstrates a camera calibration and projecting virtual 3D objects onto the video stream using Augmented reality techniques.

---

## Process for object detection
- Detecting the corners of the target, which is the checkerboard.
- Drawing those corners and storing the coordinates.
- Choosing calibration frames and saving those, calibrating the camera using those coordinates.
- Calculating the pose of the camera and validating by projecting an axis on the 1st corner.
- Building and projecting multiple objects onto the target
- Replacing the checkerboard target pattern with an image of choice (./images/sand.jpg)
- Also explored Harris and SURF features using feature.cpp file

## Important Notes
- Number of calibration frames chosen : 25
- Reprojection error after calibration : ~2.6
- Objects built : Pyramid, House, two versions of letter "S".

---

## Project Structure

```
├── bin/
│   ├── #Executable binaries
│
├── files
│   ├── # To store calibration camera matrix and distortion coefficients as csv
│
├── include/                                
│   ├── # Includes for external libraries (if any)
│
├── images/
│   ├── # The source checkerboard image, sand image which is used to replace target and calibration images.
│
├── src/                                    # Source files
│   ├── vidDisplay.cpp
│   ├── test.cpp
│   ├── utils.cpp
│   └── utils.h
│   └── feature.cpp
│
├── .gitignore                              # Git ignore file
├── makefile                                # Build configuration
├── Project4_Report.pdf                     # Project report
```

---

## Tools used
- `OS`: MacOS
- `C++ Compiler`: Apple clang version 16.0.0
- `IDE`: Visual Studio code
- `Camera source`: Iphone (Continuity camera)

---

## Dependencies
- OpenCV

**Note:** Update the dependency paths in the makefile after installation.

---

## Installation

1. Clone this repository:
   ```bash
   git clone <repository-url>
   cd <repository-folder>
   ```

2. Compile the project:
   ```bash
   make vidDisplay
   ```

3. Compile the features file:
    ```bash
    make feature
    ```

---

## Running the code

- Run the `vidDisplay` file to perform detect corners, camera calibration and project 3D objects.

- Run the `feature` file to explore Harris corners and SURF features.

---

## Usage

Key press functionality of OpenCV plays a very important role in this project. Below is all the important key press information required to understand the project.

1. For `vidDisplay.cpp`:

- `s or S`: To save the frame as a calibration frame
- `p`: To continuously project 3D axis onto the top left corner of the checkerboard pattern.
- `t`: To display hallow pyramid, house and Letter "S" with just connected edges.
- `f`: To display filled letter "S".
- `r`: To replace the target checkerboard pattern with `./images/sand.jpg` image.
- `q`: To stop the program. 

2. For `feature.cpp`:
- `s` : To use and display SURF features.
- `h` : To use and display Harris corners features.

More information about the internal implementation along with outputs is included in **[Project4_Report.pdf](https://github.com/saideep-arikontham/Calibration-and-Augmented-Reality/blob/main/Project4_Report.pdf)**



---

## Highlights
- The `utils.cpp` file includes multiple utility functions like:
    - Detecting and drawing corners
    - Projecting 3D axis
    - Creating object data
    - Projecting objects to 3D plane
    - Replace target with selected image

---

## Note

Not using any time travel days.

---

## Contact
- **Name**: Saideep Arikontham
- **Email**: arikontham.s@northeastern,edu