# ml/main_stdio_net.py

Requires to run python with `-u` flag to turn on binary stdio.

Expected stdin format:

* Intensity raster: 32x32x3 = 3072 float (each 4 bytes)
* Distance raster: 32x32x1 = 1024 float (each 4 bytes)
* Normals raster: 32x32x3 = 3072 float (each 4 bytes)
* Intensity normalization value: 1 float
* Distance normalization value: 1 float