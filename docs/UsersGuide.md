Introduction
============

This document covers a variety of topics related to working with [pbrt-v3](https://github.com/mmp/pbrt-v3),
the rendering system described in the forthcoming third edition of
*Physically Based Rendering: From Theory to Implementation*, by [Matt
Pharr](http://pharr.org/matt), Greg Humphreys, and [Wenzel
Jakob](http://www.mitsuba-renderer.org/~wenzel/).  Because most users of
pbrt are also developers who also work with the system's source code, this
guide also includes coverage of a number of topics related to the system's
structure and organization.

See also the [pbrt website](http://pbrt.org), which has general information
about both the *Physically Based Rendering* book as well as
[pbrt-v2](https://github.com/mmp/pbrt-v2), the previous version of the
system. In particular, the documentation about the [pbrt input file
format](http://pbrt.org/fileformat.html) should be useful. (Note that this
documentation still corresponds to pbrt-v2 and hasn't yet been updated for
pbrt-v3.)

If you find errors in this text or have ideas for topics that should be
discussed, please either submit a bug in the [pbrt issue
tracker](https://github.com/mmp/pbrt-v3/issues), or send an email to authors@pbrt.org.

*Note: this document is still a work in progress; a number of sections are
either unwritten or incomplete.*

Changes from pbrt-v2
====================

The system has seen many changes since the second edition. To figure out
how to use the new features, you may want to look at the example scene
files and read through the source code to figure out the parameters and
details of the following.

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
  and its build system is fully integrated with that of pbrt.

Many other small things have been improved (parallelization scheme, image
updates, statistics system, overall cleanliness of interfaces); see the
source code for details.

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

Working With The Code
=====================

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
  current directory.  Run `make -j4`, and pbrt, the obj2pbrt and imgtool
  utilities, and an executable that runs pbrt's unit tests will be built.
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
example, when running cmake from the command lne, provide it with the
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

### Porting to Different Targets ###

pbrt should compute out of the box for semi-modern versions of Linux,
FreeBSD, OpenBSD, OS X, and Windows. A C++ compiler with good support for
C++11 is required. (Therefore, pbrt definitely won't compile with any
versions of MSVC earlier than 2013, any versions of g++ before 4.8, or any
versions of clang before 3.1).

We have tried to keep as much of the system-dependent code as possible 
in the files `src/core/port.h` and `CMakeLists.txt`; ideally, only those
will need to be modified to get the system running on a new target.

We are always happy to receive patches that make it possible to build pbrt
on other targets; if you get the system buliding on a target that you think
would be useful for others, please open a pull request on github with the
changes. (Before doing so, however, please first ensure that all of the
tests run by `pbrt_test` pass on your system.)

Note that if extensive changes to pbrt are required to build it on a new
target, we may not accept the pull request, as it's also important that the
source code on github be as close as possible to the source code in the
physical book. Thus, for example, we wouldn't be interested in a pull
request that removed most of the usage of C++11 to get the system to build
with MSVC 2012 or earlier.

Branches
--------

We maintain two branches of the pbrt-v3 system. The first, "book"
corresponds as closely as possible to the source code as printed in the
physical book, *Physically Based Rendering*.  The only differences between
the two come from cases where there was a bug in the code as
published.

The second branch is available in "master". It includes all of the bug
fixes in the "book" branch, but also includes some useful new functionality
that isn't in the code in the book. In general, we don't want this branch
to diverge from the contents of the book too much, but given a sufficiently
useful new feature and in particular, given one that can be added without
substantially changing the structure of the rest of the system, we'll add
it to this branch. (It's likely that most of this additional functionality
will be included in an eventual fourth edition of the book.)  This added
code is much more extensively commented than the code that is documented in
the book.

For example, "master" now includes new files `src/core/lightdistrib.*` that
provide a better approach for sampling light sources with scenes that have
thousands of lights than the methods for sampling lights described in the
book.  We added this functionality in order to be able to efficiently
render the "Measure One" scenes in the pbrt scenes distribution, as they
include tens of thousands of emitters; the preexisting sampling methods
were unable to render these scenes without a prohibitive number of samples
per pixel.

Assertions and Logging
----------------------

The original version of pbrt (as still present in the "book" branch) uses a
custom `Assert()` macro for runtime assertions. This macro is disabled for
release builds, as some of its checks require meaningful amounts of
computation.

In the "master" branch, we have replaced `Assert()` with
[glog](https://github.com/google/glog), the Google Logging system. It
includes both improved assertion checks as well as the ability to log
system execution; we have found the logging functionality to be frequently
useful when debugging the system.

For assertions, a variety of checks are available.  The simplest is 
`CHECK()`, which is essentially a replacement for `Assert()` (or regular
`assert()` for that matter); an error and the current stack trace are
printed. There is also a `DCHECK()`, which is similar, but is only enabled
for debug builds. Furthermore, additional information about the failure can
be provided using C++ output streams:
```
CHECK(foo.x == bar.y) &lt;&lt; "Unexpected inequality: foo = " &lt;&lt; foo &lt;&lt; ", bar = " &lt;&lt; bar;
```


When possible, it's better to use macros that separately take the value
being checked and the expected value. For example, `CHECK_EQ()` takes two
values and tests whether they are equal, printing an error if not. It
includes the improvement over `CHECK()` that the two values are printed as
well upon failure. Similarly, `CHECK_NE()` checks for inequality, and
`CHECK_LT()`, `CHECK_LE()`, `CHECK_GT()`, and `CHECK_GE()` check for less
than, less than or equal, greater than, and greater than or equal,
respectively.  There are debug-only variants of these that also are
prefixed with "D".

The glog system also includes a rich logging infrastructure, which makes it
possible information about "interesting" program events. In addition to the
in addition to the string provided by the user, a fine-grained time stamp
and thread id are included in the log, which are also often helpful:
```
LOG(INFO) << "Starting film tile " << tileBounds;
```
There are four logging levels: `INFO`, `WARNING`, `ERROR`, and `FATAL`.
Normally logs are stored in the system temporary directory (either as
specified by the `TMPDIR` environment variable, or in a system default like
`/tmp`), but this can be overridden using pbrt's `--logdir` command-line
option.  pbrt has the command line option `--logtostderr`, which causes logging
information to be printed to standard error at runtime.

(Note that on OSX, `TMPDIR` is set to a unique per-session temporary
directory; you won't find your logs if you look for them in `/tmp`.)

We have removed the `Info()` and `Severe()` functions from pbrt (which were
essentially used for this form of logging).  However, we have retained
`Warning()` and `Error()`: the distinction is that those two continue to be
used for user-facing error messages (parameters not recognized in input
files and so forth), while the logging functionality is used only for
information that is only useful for developers.

Sometimes verbose logging is useful for debugging. glog also offers
`VLOG()`, which takes an integer logging level as an argument.  Then, the
`--v` command-line option to pbrt can be used to enable various verbose
logging levels.

Debugging
---------

Debugging a ray-tracer can be its own special kind of fun. When the system
crashes, it may take hours of computation to reach the point where the
crash occurs. When an incorrect image is generated, chasing down why it is
that the image is usually-but-not-always correct can be a very tricky
exercise.

XXX avoiding bugs in the first place: -Wall, /analyze, clang-check

When trouble strikes, it's usually best to start by rendering the scene
again using a debug build of pbrt.  Debug builds of the system not only
include debugging symbols and aren't highly optimized (so that the program
can be effectively debugged in a debugger), but also include more runtime
assertions, which may help narrow down the issue. We find that debug builds
are generally three to five times slower than optimized builds; thus, if
you're not debugging, make sure you're not using a debug build!  (See
Section [Debug and Release Builds] for information about how to create a
debug build.)

(Depending on how easy it is to work with multi-threaded programs in your
debugger, you may find it helpful to give pbrt `--nthreads=1` as a
command-line argument when debugging.  Program execution will be even
slower, but tracing program execution in a single thread is often
easier. However, if a bug disappears when you use a single thread, you may
have a data race or other multithreading-related issue.)

One of the best cases is if an assertion is failing. This at least gives an
initial clue to the problem--assuming that the assertion is not itself
buggy, then the debugging task is just to figure out the sequence of events
that led to it failing. In general, we've found that taking the time to add
carefully-considered assertions to the system more than pays off in terms
of reducing future time working backward when things go wrong.

If the system crashes outright (e.g., with a segmentation violation), then
the task is to see what memory access caused the problem. From here, XXX.

valgrind, asan, msan

We have found that the
[helgrind](http://valgrind.org/docs/manual/hg-manual.html) tool to be very
useful for debugging threading-related bugs. (See also [Thread
Sanitizer](http://clang.llvm.org/docs/ThreadSanitizer.html), XXXX.)

Debugging image errors

For help with chasing down more subtle errors, many of the classes in the
system both have implementations of `operator<<` for printing to C++
ostreams and also have a `ToString()` method that returns a std::string
describing the values stored in an object. Both of these can be useful when
printing out debugging information, possibly in conjunction with the
logging system described in Section [Assertions and Logging].  (Note,
however, that output will be garbled if multiple threads concurrently print
to std::cout or std::cerr; either use a single thread, or use the logging
mechanism.) If you find it useful to add these methods to a class that
doesn't currently have them implemented, please send us a pull request with
the corresponding changes so that they can be made available to other pbrt
users.

For images of complex scenes with many samples per pixel, it may be
necessary for pbrt to run for a long time before it gets to the point where
a bug manifests itself, which in turn can make debugging slow going.  If
you can figure out the pixel coordinates of the pixel where a bug occurs,
there is a "pixelbounds" parameter that limits rendering to a given range
of pixels that can greatly speed up this process; this parameter is
supported by all of the integrators other than `SPPMIntegrator` and
`MLTIntegrator`.

The pixel bounds are specified by four integer values, corresponding (in
order) to starting x, ending x, starting y, and ending y pixel
coordinates. If specified, pbrt will do minimal work for the other image
pixels, but will ensure (as much as possible) that the program state is
consistent with how it would be for those pixels if the entire image was
rendered. (Note that the `RandomSampler` won't have the correct starting
state if this parameter is used, though this sampler generally shouldn't be
used anyway.)

Unit Tests
----------

We have written unit tests for some parts of the system (primarily for new
functionality added with pbrt-v3, but some to test pre-existing
functionality). Running the `pbrt_test` executable, which is built as part
of the regular build process, causes all tests to be executed. Unit tests
are written using the [Google C++ Testing
Framework](https://github.com/google/googletest), which is included with
the pbrt distribution.
See the [Google Test
Primer](https://github.com/google/googletest/blob/master/googletest/docs/Primer.md)
for more information about how to write new test.
 
We have found these tests to be quite useful when developing new features,
testing the system after code changes, and when porting the system to new
targets.  Over time, we plan to add more tests and are also always happy to
receive pull requests with additions to the system's tests.


Extending pbrt
--------------

TODO

### Adding a new BxDF ###


Pull Requests
-------------

We're always happy to get pull requests or patches that improve
pbrt. However, we are unlikely to accept pull requests that significantly
change the system's structure, as we don't want the "master" branch to
diverge *too* far from the contents of the book.  (In this cases, however,
we certainly encourage you to maintain a separate fork of the system on
github and to let people know about it on the pbrt mailing list.)

Pull requests that fix bugs in the system or improve portability are always
welcome. (If you have an itch to write unit tests and add new ones to
`src/tests`, we're also happy to expand the test coverage of the system!)
Finally, if you write a converter that makes it easier to export scenes
from other file formats or modeling systems to pbrt's format, we'd happily
include it in the pbrt distribution.


Rendering with pbrt
===================

Example Scenes
--------------

Over 4GB of example scenes are available for download. (Many are new and
weren't available with previous versions of pbrt.) We're trying an
experiment and making them available via git. Run:
```
$ git clone git://git.pbrt.org/pbrt-v3-scenes
```
to get them. We will update this repository as more scenes become
available. (See the `README.md.html file` in the scene distribution for
more information about the scenes, preview images, and information about
how to submit new and/or improved scenes to that distribution.)

A number of very nice scenes for pbrt-v3 are also available from Benedikt
Bitterli's [rendering resources](https://benedikt-bitterli.me/resources/)
webpage.


Working with Rendered Images
----------------------------

note that preview sucks with exr files

imgtool

tone mapping and bloom


Rendering Previews
------------------

It's often useful to quickly generate a 

Choosing an Integrator
----------------------

Noise
-----

Spiky noise: bad

Otherwise, increase pixel samples..

Distributed Rendering
---------------------

Cropwindow

imgtool assemble


Converting Scenes to pbrt's Format
----------------------------------

See the README in the example scene distribution...
