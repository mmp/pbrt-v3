
/*
    pbrt source code is Copyright(c) 1998-2016
                        Matt Pharr, Greg Humphreys, and Wenzel Jakob.

    This file is part of pbrt.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:

    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
    IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */


// main/pbrt.cpp*
#include "pbrt.h"
#include "api.h"
#include "parser.h"
#include "parallel.h"
#include <glog/logging.h>

using namespace pbrt;

static void usage(const char *msg = nullptr) {
    if (msg)
        fprintf(stderr, "pbrt: %s\n\n", msg);

    fprintf(stderr, R"(usage: pbrt [<options>] <filename.pbrt...>
Rendering options:
  --help               Print this help text.
  --nthreads <num>     Use specified number of threads for rendering.
  --outfile <filename> Write the final image to the given filename.
  --quick              Automatically reduce a number of quality settings to
                       render more quickly.
  --quiet              Suppress all text output other than error messages.
  --reference=<nTiles>
                       Enables the reference mode with nTiles per dimension
  --reference_samples=<nsamples>
                       Sets the number of samples for the reference
                       path tracer
  --reference_resume=<0|1>
                       With resume disabled, output directory is not checked
                       and existing files will be overwritten
                       When it's enabled, existing files will be skipped and
                       not re-rendered
  --iispt_hemi_size=<pixel>
                       Set the dimension of the IISPT hemispherical renders
                       Defaults to 32


Logging options:
  --logdir <dir>       Specify directory that log files should be written to.
                       Default: system temp directory (e.g. $TMPDIR or /tmp).
  --logtostderr        Print all logging messages to stderr.
  --minloglevel <num>  Log messages at or above this level (0 -> INFO,
                       1 -> WARNING, 2 -> ERROR, 3-> FATAL). Default: 0.
  --v <verbosity>      Set VLOG verbosity.

Reformatting options:
  --cat                Print a reformatted version of the input file(s) to
                       standard output. Does not render an image.
  --toply              Print a reformatted version of the input file(s) to
                       standard output and convert all triangle meshes to
                       PLY files. Does not render an image.
)");
}




// ============================================================================
// Test section

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/wait.h>
#include <chrono>
#include <thread>

#include "tools/childprocess.hpp"

static void test_main2() {
    std::cerr << "Running test main 2" << std::endl;

    char *const argv[] = {"python3", "-u", "/home/gj/git/pbrt-v3-IISPT/tools/test_python_child_bin.py", NULL};
    ChildProcess cp (std::string("python3"), argv);
    cp.write_float32(595.0);
    Float fresult;
    int status = cp.read_float32(&fresult);
    if (status) {
        std::cerr << "Error when reading the float" << std::endl;
    } else {
        std::cerr << "Got a float: " << fresult << std::endl;
    }

    status = cp.read_float32(&fresult);
    if (status) {
        std::cerr << "Reading got nothing" << std::endl;
    }

    exit(0);
}

static void test_main1() {
    std::cerr << "Running test main 1" << std::endl;

    char *const argv[] = {"python3", "-u", "/home/gj/git/pbrt-v3-IISPT/tools/test_python_child.py", NULL};
    ChildProcess cp (std::string("python3"), argv);
    cp.write_char('b');
    cp.write_char('\n');
    while (1) {
        char r = cp.read_char();
        if (r == '\0') {
            exit(0);
        }
        std::cerr << "["<< r <<"]" << std::endl;
    }
}

static void test_main0() {
    std::cerr << "Running test main" << std::endl;

    pid_t pid;
    int stdout_pipe[2];
    int stdin_pipe[2];

    pipe(stdout_pipe);
    pipe(stdin_pipe);
    pid = fork();

    if (pid == -1) {
        std::cerr << "fork() failed" << std::endl;
        exit(1);
    } if (pid==0) {
        // Child process

        dup2(stdout_pipe[1], STDOUT_FILENO); // Subprocess receives write end of stdout pipe
        dup2(stdin_pipe[0], STDIN_FILENO); // Subprocess receives read end of stdin pipe
        close(stdout_pipe[0]);
        close(stdout_pipe[1]);
        close(stdin_pipe[0]);
        close(stdin_pipe[1]);

        execlp("python3", "python3", "-u", "/home/gj/git/pbrt-v3-IISPT/tools/test_python_child.py", NULL); // Should never return in case of success

        std::cerr << "Failed execlp()" << std::endl;

        exit(1);
    }

    // Original process
    std::cerr << "This is the original process" << std::endl;

    int buffsize = 100;
    char buffer[buffsize];
    write(stdin_pipe[1], "b\n", 2);
    while (1) {
        std::cerr << "Reading..." << std::endl;
        ssize_t count = read(stdout_pipe[0], buffer, sizeof(buffer)); // Read from stdout's read end
        std::cerr << "Read count is " << count << std::endl;
        if (count <= 0) {
            std::cerr << "EOF" << std::endl;
            break;
        } else {
            std::cerr << "From child: [" << buffer << "]" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cerr << "Parent: writing a" << std::endl;
        write(stdin_pipe[1], "a\n", 2);
    }

    exit(0);
}

// ============================================================================
// Main
int main(int argc, char *argv[]) {
    // test_main2();

    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = 1; // Warning and above.

    Options options;
    std::vector<std::string> filenames;
    // Process command-line arguments
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--nthreads") || !strcmp(argv[i], "-nthreads")) {
            if (i + 1 == argc)
                usage("missing value after --nthreads argument");
            options.nThreads = atoi(argv[++i]);
        } else if (!strncmp(argv[i], "--nthreads=", 11)) {
            options.nThreads = atoi(&argv[i][11]);
        } else if (!strcmp(argv[i], "--outfile") || !strcmp(argv[i], "-outfile")) {
            if (i + 1 == argc)
                usage("missing value after --outfile argument");
            options.imageFile = argv[++i];
        } else if (!strncmp(argv[i], "--outfile=", 10)) {
            options.imageFile = &argv[i][10];
        } else if (!strcmp(argv[i], "--logdir") || !strcmp(argv[i], "-logdir")) {
            if (i + 1 == argc)
                usage("missing value after --logdir argument");
            FLAGS_log_dir = argv[++i];
        } else if (!strncmp(argv[i], "--logdir=", 9)) {
            FLAGS_log_dir = &argv[i][9];
        } else if (!strcmp(argv[i], "--minloglevel") ||
                   !strcmp(argv[i], "-minloglevel")) {
            if (i + 1 == argc)
                usage("missing value after --minloglevel argument");
            FLAGS_minloglevel = atoi(argv[++i]);
        } else if (!strncmp(argv[i], "--minloglevel=", 14)) {
            FLAGS_minloglevel = atoi(&argv[i][14]);
        } else if (!strcmp(argv[i], "--quick") || !strcmp(argv[i], "-quick")) {
            options.quickRender = true;
        } else if (!strcmp(argv[i], "--quiet") || !strcmp(argv[i], "-quiet")) {
            options.quiet = true;
        } else if (!strcmp(argv[i], "--cat") || !strcmp(argv[i], "-cat")) {
            options.cat = true;
        } else if (!strcmp(argv[i], "--toply") || !strcmp(argv[i], "-toply")) {
            options.toPly = true;
        } else if (!strcmp(argv[i], "--v") || !strcmp(argv[i], "-v")) {
            if (i + 1 == argc)
                usage("missing value after --v argument");
            FLAGS_v = atoi(argv[++i]);
        } else if (!strncmp(argv[i], "--v=", 4)) {
          FLAGS_v = atoi(argv[i] + 4);
        }
        else if (!strcmp(argv[i], "--logtostderr")) {
          FLAGS_logtostderr = true;
        } else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-help") ||
                   !strcmp(argv[i], "-h")) {
            usage();
            return 0;
        }
        else if (!strncmp(argv[i], "--reference=", 12)) {
            fprintf(stderr, "Detected --reference= option\n");
            options.referenceTiles = atoi(&argv[i][12]);
            std::cerr << "Set reference tiles to " << options.referenceTiles << std::endl;
        }
        else if (!strncmp(argv[i], "--reference_samples=", 20)) {
            options.referencePixelSamples = atoi(&argv[i][20]);
            std::cerr << "Set reference samples to " << options.referencePixelSamples << std::endl;
        }
        else if (!strncmp(argv[i], "--reference_resume=", 19)) {
            options.referenceResume = atoi(&argv[i][19]);
            std::cerr << "Set reference resume to " << options.referenceResume << std::endl;
        }
        else if (!strncmp(argv[i], "--iispt_hemi_size=", 18)) {
            options.iisptHemiSize = atoi(&argv[i][18]);
            std::cerr << "Set IISPT hemi size to " << options.iisptHemiSize << std::endl;
        }
        else {
            filenames.push_back(argv[i]);
        }
    }

    // Print welcome banner
    if (!options.quiet && !options.cat && !options.toPly) {
        if (sizeof(void *) == 4)
            printf("*** WARNING: This is a 32-bit build of pbrt. It will crash "
                   "if used to render highly complex scenes. ***\n");
        printf("pbrt version 3 (built %s at %s) [Detected %d cores]\n",
               __DATE__, __TIME__, NumSystemCores());
#ifndef NDEBUG
        LOG(INFO) << "Running debug build";
        printf("*** DEBUG BUILD ***\n");
#endif // !NDEBUG
        printf(
            "Copyright (c)1998-2018 Matt Pharr, Greg Humphreys, and Wenzel "
            "Jakob.\n");
        printf(
            "The source code to pbrt (but *not* the book contents) is covered "
            "by the BSD License.\n");
        printf("See the file LICENSE.txt for the conditions of the license.\n");
        fflush(stdout);
    }
    pbrtInit(options);
    // Process scene description
    if (filenames.empty()) {
        // Parse scene from standard input
        ParseFile("-");
    } else {
        // Parse scene from input files
        for (const std::string &f : filenames)
            ParseFile(f);
    }
    pbrtCleanup();
    return 0;
}
