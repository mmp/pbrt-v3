pbrt, Version 3
===============

[![Build Status](https://travis-ci.org/mmp/pbrt-v3.svg?branch=master)](https://travis-ci.org/mmp/pbrt-v3)
[![Build status](https://ci.appveyor.com/api/projects/status/mlm9g91ejxlcn67s/branch/master?svg=true)](https://ci.appveyor.com/project/mmp/pbrt-v3/branch/master)

This repository holds the source code to the version of pbrt that is
described in the third edition of *Physically Based Rendering: From
Theory to Implementation*, by [Matt Pharr](http://pharr.org/matt), [Wenzel
Jakob](http://www.mitsuba-renderer.org/~wenzel/), and Greg Humphreys.  As
before, the code is available under the BSD license.

The [pbrt website](http://pbrt.org) has general information about
both the *Physically Based Rendering* book as well as many other resources
for pbrt.

Example scenes
--------------

Over 8GB of example scenes are available for download. (Many are new and
weren't available with previous versions of pbrt.)  See the [pbrt-v3 scenes
page](http://pbrt.org/scenes-v3.html) on the pbrt website for information
about how to download them.

After downloading them, see the `README.md.html` file in the scene
distribution for more information about the scenes and preview images.

Additional resources
--------------------

* There is a [pbrt Google
  Groups](https://groups.google.com/forum/#!forum/pbrt) mailing list that can
  be a helpful resource.
* Please see the [User's Guide](http://pbrt.org/users-guide.html) for more
  information about how to check out and build the system as well as various
  additional information about working with pbrt.
* Should you find a bug in pbrt, please report it in the [bug
  tracker](https://github.com/mmp/pbrt-v3/issues).
* Please report any errors you find in the *Physically Based Rendering*
  book to authors@pbrt.org.

Note: we tend to let bug reports and book errata emails pile up for a few
months for processing them in batches. Don't think we don't appreciate
them. :-)

Building pbrt
-------------

To check out pbrt together with all dependencies, be sure to use the
`--recursive` flag when cloning the repository, i.e.
```bash
$ git clone --recursive https://github.com/mmp/pbrt-v3/
```
If you accidentally already cloned pbrt without this flag (or to update an
pbrt source tree after a new submodule has been added, run the following
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
  `cmake [path to pbrt-v3]`. A Makefile will be created in that
  current directory.  Run `make -j8`, to build pbrt, the obj2pbrt and imgtool
  utilities, and an executable that runs pbrt's unit tests.
* To make an XCode project file on OS X, run `cmake -G Xcode [path to pbrt-v3]`.
* Finally, on Windows, the cmake GUI will create MSVC solution files that
  you can load in MSVC.

If you plan to edit the lexer and parser for pbrt's input files
(`src/core/pbrtlex.ll` and `src/core/pbrtparase.y`), you'll also want to
have [bison](https://www.gnu.org/software/bison/) and
[flex](http://flex.sourceforge.net/) installed. On OS X, note that the
version of flex that ships with the developer tools is extremely old and is
unable to process `pbrtlex.ll`; you'll need to install a more recent
version of flex in this case.

### Debug and Release Builds ###

By default, the build files that are created that will compile an optimized
release build of pbrt. These builds give the highest performance when
rendering, but many runtime checks are disabled in these builds and
optimized builds are generally difficult to trace in a debugger.

To build a debug version of pbrt, set the `CMAKE_BUILD_TYPE` flag to
`Debug` when you run cmake to create build files to make a debug build. For
example, when running cmake from the command line, provide it with the
argument `-DCMAKE_BUILD_TYPE=Debug`. Then build pbrt using the resulting
build files. (You may want to keep two build directories, one for release
builds and one for debug builds, so that you don't need to switch back and
forth.)

Debug versions of the system run much more slowly than release
builds. Therefore, in order to avoid surprisingly slow renders when
debugging support isn't desired, debug versions of pbrt print a banner
message indicating that they were built for debugging at startup time.

### Build Configurations ###

There are two configuration settings that must be set at compile time. The
first controls whether pbrt uses 32-bit or 64-bit values for floating-point
computation, and the second controls whether tristimulus RGB values or
sampled spectral values are used for rendering.  (Both of these aren't
amenable to being chosen at runtime, but must be determined at compile time
for efficiency).

To change them from their defaults (respectively, 32-bit
and RGB.), edit the file `src/core/pbrt.h`.

To select 64-bit floating point values, remove the comment symbol before
the line:
```
//#define PBRT_FLOAT_AS_DOUBLE
```
and recompile the system.

To select full-spectral rendering, comment out the first of these two
typedefs and remove the comment from the second one:
```
typedef RGBSpectrum Spectrum;
// typedef SampledSpectrum Spectrum;
```
Again, don't forget to recompile after making this change.

