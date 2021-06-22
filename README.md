# asl_fixedwing
Fixed Wing Aircraft Control

![Fixed Wing Image](./img/fixedwing.gif)

# Setup
Start by cloning this repository into your ROS1 workspace.
```
cd <ros1_ws/src>
git clone https://github.com/StanfordASL/asl_fixedwing.git
```

## QP Solver
Use qpOASES as the QP solver. Note we are using the asl-v3.2.0 branch instead of the asl-v3.2.1 branch due to an unresolved issue with linking against the qpOASES library with v3.2.1.

```
cd ~
git clone https://github.com/jlorenze/qpOASES.git
cd qpOASES
git checkout asl-v3.2.0
mkdir bin
make
```

Then update the CMakeLists.txt file in this repository to make sure it has the line:
```
SET(QPOASES_INCLUDE_DIRS <path-to-install-dir>)
SET(QPOASES_LIBRARIES <path-to-install-dir>/bin/libqpOASES.a
					${LAPACK_LIBRARIES}
					${BLAS_LIBRARIES})
```
Now build the package
```
source <ros1_ws/devel/setup.bash
catkin build asl_fixedwing
```
