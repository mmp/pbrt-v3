pbrt, Version 3
===============

[![Build Status](https://travis-ci.org/mmp/pbrt-v3.svg?branch=master)](https://travis-ci.org/mmp/pbrt-v3)
[![Build status](https://ci.appveyor.com/api/projects/status/mlm9g91ejxlcn67s?svg=true)](https://ci.appveyor.com/project/mmp/pbrt-v3)

This repository holds the source code to the new version of pbrt that will
be described in the forthcoming third edition of *Physically Based
Rendering: From Theory to Implementation*, by [Matt
Pharr](http://pharr.org/matt), [Wenzel
Jakob](http://www.mitsuba-renderer.org/~wenzel/), and Greg Humphreys.  As
before, the code is available under the BSD license.

Although the new version of the book won't be released until this Fall,
we're making the source code available now so that interested users can
look at the code, try out the system, and possibly help us out. (See [how
you can help](#how-you-can-help) for more information about contributing.)
The initial release of the source code doesn't include updated
documentation (and the book isn't out yet!), so you should only try it out
if you're comfortable digging into source code.

Over 4GB of example scenes are available for download. (Many are new and
weren't available with previous versions of pbrt.) We're trying an
experiment and making them available via git. Run:
```
$ git clone git://git.pbrt.org/pbrt-v3-scenes
```
to get them. We will update this repository as more scenes become
available. (See the `README.md.html file` in the scene distribution for
more information about the scenes and preview images.)

The [pbrt website](http://pbrt.org) has  general information about
both *Physically Based Rendering* as well as pbrt-v2, the previous version
of the system.

Significant Changes
-------------------

The system has seen many changes since the second edition. To figure out
how to use the new features, you may want to look at the example scene
files and read through the source code to figure out the parameters and
details of the following. (Better documentation will come once everything
is finalized.)

* Bidirectional path tracing: `Integrator "bdpt"` does proper bidirectional
  path tracing with multiple importance sampling.
* Metropolis sampling: `Integrator "mlt"` uses the bidirectional path
  tracer with Hachisuka et al.'s "Multiplexed Metropolis Light Transport"
  technique.
* Improved numerical robustness for intersection calculations: epsilons are
  small and provably conservative. [Section
  draft](http://pbrt.org/fp-error-section.pdf)
* Subsurface scattering: all new implementation, integrated into the path
  tracing integrator.  See the `scenes/head` example scene.
* Curve shape: thin ribbons described by bicubic Bezier curves. Great for
  hair, fur, and grass.
* PLY mesh support: meshes in PLY format can be used directly: `Shape
  "plymesh" "string filename" "mesh.ply"`
  * Existing scenes with triangle meshes specified via `Shape
    "trianglemesh"` can be converted to PLY using the `--toply` command-line
    option, which emits a PLY mesh for each triangle mesh and prints an
    updated scene description file to standard out.
* Realistic camera model: tracing rays through lenses to make images! See
  the `scenes/dragons` example scene.
* Participating media: the boundaries of shapes are now used to delineate
  the extent of regions of participating media in the scene.  See the
  `scenes/medium-sphere` example scene.
* New samplers: a much-improved Halton sampler, and an all-new Sobol'
  sampler are both quite effective for path tracing and bidirectional path
  tracing.
* Fourier representation of measured materials: an implementation of Jakob
  et al's [A Comprehensive Framework for Rendering Layered
  Materials](http://www.cs.cornell.edu/projects/layered-sg14/). (See an
  example in the `scenes/dragons` example scene).
  * New versions of the BSDF files it uses can be generated with 
    [layerlab](https://github.com/wjakob/layerlab).
* Improved microfacet models: specular transmission through microfacets,
  and Heitz's improved importance sampling.
* No external dependencies: thanks to
[Lode Vandevenne's lodepng](http://lodev.org/lodepng/),
[Diego Nehab's rply](http://www.impa.br/~diego/software/rply),
and [Emil Mikulic's TARGA library](http://dmr.ath.cx/gfx/targa/), no
  external libraries need to be compiled to build pbrt.
  The only slightly bigger dependency is [OpenEXR](http://www.openexr.com/),
  and its build system is fully integrated with that of PBRT.

Many other small things have been improved (parallelization scheme, image
updates, statistics system, overall cleanliness of interfaces); see the
source code for details.

Building The System
-------------------

First, to check out pbrt together with all dependencies, be sure to use the ``--recursive`` flag
when cloning the repository, i.e.
```bash
$ git clone --recursive https://github.com/mmp/pbrt-v3/
```
If you accidentally already cloned pbrt without this flag (or to update an
pbrt source tree from before change ``b9aa97b1f21f36b0``, run the following
command to also fetch the dependencies:
```bash
$ git submodule update --init --recursive
```

pbrt uses [cmake](http://www.cmake.org/) for its build system.  On Linux
and OS X, cmake is available via most package management systems.  For
Windows, or to build it from source, see the [cmake downloads
page](http://www.cmake.org/download/).

* For command-line builds on Linux and OS X, once you have cmake installed,
create a new directory for the build, change to that directory, and run
`cmake <path to pbrt-v3>`. A Makefile will be created in that
current directory.  Run `make -j4`, and pbrt and some additional tools will
be built.
* To make an XCode project file on OS X, run `cmake -G Xcode <path to
pbrt-v3>`.
* Finally, on Windows, the cmake GUI will create MSVC solution files that
you can load in MSVC.

File Format Changes
-------------------

We've tried to keep the scene description file format as unchanged as
possible.  However, progress in other parts of the system required changes
to the scene description format.

First, the "Renderer", "SurfaceIntegrator", and "VolumeIntegrator"
directives have all been unified under "Integrator"; only a single
integrator now needs to be specified.

The following specific implementations were removed:

* Accelerator "grid"
  * Use "bvh" or "kdtree" instead.
* Material "measured", "shinymetal"
  * The new "fourier" material should now be used for measured BRDFs.
  * Use "uber" in place of "shinymetal".
* VolumeRegion: all
  * Use the new participating media representation [described above](#significant-changes)
* SurfaceIntegrator: photonmap, irradiancecache, igi, dipolesubsurface,
  ambientocclusion, useprobes, diffuseprt, glossyprt
  * Use "sppm" for the "photonmap".
  * The "path" integrator now handles subsurface scattering directly.
  * The others aren't as good as path tracing anyway. :-)
* VolumeIntegrator: single, emission
  * Use the "volpath" path tracing integrator.
* Sampler: bestcandidate
  * Use any other sampler.
* Renderer: all
  * Use "Integrator", as described above.
