# ml/main_stdio_net.py

Requires to run python with `-u` flag to turn on binary stdio.

Expected stdin format:

* Intensity raster: 32x32x3 = 3072 float (each 4 bytes)
* Distance raster: 32x32x1 = 1024 float (each 4 bytes)
* Normals raster: 32x32x3 = 3072 float (each 4 bytes)
* Intensity normalization value: 1 float
* Distance normalization value: 1 float

# Saved images and PBRT internal image representation

In PBRT, images coordiantes X and Y:

* X from left to right
* Y from bottom to top

But the saved images follow

* Y from top to bottom

In the datasets:

* d - uses PBRT exporter
* n - uses custom exporter, Y is reversed
* z - uses custom exporter, Y is reversed
* p - uses PBRT exporter, Y is reversed

The ImageFilm object handles the Y direction in the same way as PBRT, maintaining consistency with the training dataset. Therefore once an ImageFilm object is obtained, there is no need to handle manual transformations. Normals and Distance keep their inverted axis, the neural network handles the axes automatically.