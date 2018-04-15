# ml/main_stdio_net.py

Requires to run python with `-u` flag to turn on binary stdio.

Expected stdin format:

* Intensity raster: 32x32x3 = 3072 float (each 4 bytes)
* Distance raster: 32x32x1 = 1024 float (each 4 bytes)
* Normals raster: 32x32x3 = 3072 float (each 4 bytes)
* Intensity normalization value: 1 float
* Distance normalization value: 1 float

Expected stdout format:

* Intensity raster: 32x32x3 = 3072 float (each 4 bytes)
* Magic characters sequence: 'x' '\n'

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

The IntensityFilm object handles the Y direction in the same way as PBRT, maintaining consistency with the training dataset. Therefore once an IntensityFilm object is obtained, there is no need to handle manual transformations. Normals and Distance keep their inverted axis, the neural network handles the axes automatically.

# Environment Variables

`IISPT_STDIO_NET_PY_PATH` Location of `main_stdio_net.py` file which contains the python program to evaluate the neural network. Used by PBRT to start the child process. The environment variable is set up by the pbrt launcher.

`IISPT_SCHEDULE_RADIUS_START` Initial radius.

`IISPT_SCHEDULE_RADIUS_RATIO` Radius update multiplier.

`IISPT_SCHEDULE_INTERVAL` Radius interval samples.

# IISPT Render Algorithm

## Classes

### IisptRenderRunner

One render thread. It includes the main loop logic.

Requires shared objects:

* IISPTIntegrator
* IisptScheduleMonitor
* IisptFilmMonitor (includes sample density information)

It creates its own instance of:

* IISPTdIntegrator
* IisptNnConnector (requires `dcamera` and `scene`)
* RNG

The render loop works as follows

* Obtain current __radius__ from the __ScheduleMonitor__. The ScheduleMonitor updates its internal count automatically
* Use the __RNG__ to generate 2 random pixel samples. Look up the density of the samples and select the one that has lower density
* Obtain camera ray and shoot into scene. If no __intersection__ is found, evaluate infinite lights
* Create __auxCamera__ and use the __dIntegrator__ to render a view
* Use the __NnConnector__ to obtain the predicted intensity
* Set the predicted intensity map on the __auxCamera__
* Create a __filmTile__ in the radius section
* For all pixels within __radius__ and whose intersection and materials are compatible with the original intersection, evaluate __Li__ and update the filmTile
* Send the filmTile to the __filmMonitor__

### IisptScheduleMonitor

Maintains the schedule of influence radius and radius update interval.

The radius schedule uses 2 parameters:

* Initial radius. Defaults to 50, overridden by `IISPT_SCHEDULE_RADIUS_START`
* Update multiplier. Defaults to 0.90, overridden by `IISPT_SCHEDULE_RADIUS_RATIO`

When radius is <= 1, only the original pixel is affected.

The radius update interval is the number of IISPT samples generated after the radius changes. A sample is considered to be generated at each call to __get_current_radius()__.

Defaults to 500, overridden by `IISPT_SCHEDULE_INTERVAL`.

### IisptFilmMonitor

Represents the full rendering film used by IISPT.

All the coordinates in the public API are absolute x and y coordinates, and are converted to internal film indexes automatically.

Holds a 2D array of __IisptPixel__.

__TODO__ This replaces the old IisptFilmMonitor class

Public methods:

* constructor(Bounds2i)
* add_sample(int x, int y, Spectrum s)
* get_density(int x, int y)

### IisptPixel

An IisptPixel has:

* x, y, z color coordinates
* sample_count number of samples obtained at the current location