collada_urdf_jsk_patch
======================

This is a yet another version of [collada_urdf](http://wiki.ros.org/collada_urdf) package.

Original `collada_urdf` depends on `libassimp` package provided by Ubuntu Update and has a serious bug on visual when spawning exported URDF file into Gazebo.

This patched version solved this problem using newer `assimp` library (>= `3.1`).

## For Developers

This package depends on `assimp_devel` which provides newer version of `libassimp` (`3.1.1`).

Current ubuntu distributions using assimp older than `3.1` must use this package instead of original `collada_urdf` and `libassimp`.


- Ubuntu 12.04 LTS: `2.0.863+dfsg-2` (< 3.1)
- Ubuntu 14.04 LTS: `3.0~dfsg-2` (< 3.1)
- Ubuntu 16.04 LTS: `3.2~dfsg-3` (>= 3.1)

We will be able to disable this package and `assimp_devel` on kinetic (because kinetic supports 16.04 or newer).
