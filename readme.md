# srt3d_js

Emscripten version for [*SRT3D: Region-Based 6DoF Object Tracking*](https://github.com/DLR-RM/3DObjectTracking/tree/master/SRT3D)


https://github.com/user-attachments/assets/ce663575-67e7-4d38-b25f-748b86e890e7



## Modification
#### 1. Automatical body-diameter calculation
You don't need to specify `max_body_diameter` manually, just as [*M3T*](https://github.com/DLR-RM/3DObjectTracking/tree/master/M3T).

#### 2. Kullback-Leibler divergence based confidence assessment 
If the predicted pose is good (close to the reality), the mean value of each correspondence line will be zero (at the center, without offset). We calculate KL divergence for each correspodence line and the optimal one: $\mathcal{N}(0, \sigma_{min}^2)$.

#### 3. Thresholds for initialization and tracking
We define thresholds for initialization and tracking (you can manually set them), so that you can easily align the object to the virtual one before start tracking.

#### 4. Independent multi-model tracing
The confidences are calculated independently. You can easily add models and get their information (pose, conf., etc.).


## Installation
Make sure that you have installed the following packages:
- OpenGL
- GLEW
- glfw3
- Eigen3
- OpenCV 3/4

And the following packages under `Emscripten`
- OpenCV 3/4
- Eigen3


Your compilation environment should support `C++ 17`.

```
## compile template renderer
cd ${repo_root}/template_renderer
mkdir build && cd build
cmake ..
make

## compile srt3d_js
# activate your emscripten environment first
cd ${repo_root}/srt3d_js
mkdir build && cd build
emcmake cmake ..
emmake make
```


## Demo
\* *The demo data is from [DeepAC](https://github.com/WangLongZJU/DeepAC)*.
Open `example/index.html` and click the *Play* button for the video.


## Interface
Just follow `example/index.html` or `srt3d_js/srt3d_js/srt3d_js.cpp`.
