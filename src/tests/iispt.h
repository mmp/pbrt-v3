#ifndef IISPT_H
#define IISPT_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/wait.h>
#include <chrono>
#include <thread>

#include "tools/childprocess.hpp"
#include "integrators/iisptschedulemonitor.h"
#include "integrators/iisptfilmmonitor.h"

using namespace pbrt;

void test_main4() {
    std::cerr << "Running test main 4: IisptFilmMonitor" << std::endl;

    std::shared_ptr<IisptFilmMonitor> film_monitor (
                new IisptFilmMonitor(
                    Bounds2i(
                        Point2i(10, 10),
                        Point2i(100, 50)
                        )
                    )
                );

    // Draw a square in section (10, 10) -> (40, 40)
    for (int y = 10; y <= 40; y++) {
        for (int x = 10; x <= 40; x++) {
            float rgb[3];
            rgb[0] = x;
            rgb[1] = y;
            rgb[2] = 0.0;
            film_monitor->add_sample(
                        Point2i(x, y),
                        Spectrum::FromRGB(rgb)
                        );
        }
    }

    // Get some density values
    int x;
    int y;
    int d;

    x = 10;
    y = 10;
    d = film_monitor->get_pixel_sampling_density(x, y);
    std::cerr << "Density at ["<< x <<"] ["<< y <<"] is ["<< d <<"]" << std::endl;

    x = 10;
    y = 10;
    d = film_monitor->get_pixel_sampling_density(x, y);
    std::cerr << "Density at ["<< x <<"] ["<< y <<"] is ["<< d <<"]" << std::endl;

    x = 40;
    y = 40;
    d = film_monitor->get_pixel_sampling_density(x, y);
    std::cerr << "Density at ["<< x <<"] ["<< y <<"] is ["<< d <<"]" << std::endl;

    x = 41;
    y = 41;
    d = film_monitor->get_pixel_sampling_density(x, y);
    std::cerr << "Density at ["<< x <<"] ["<< y <<"] is ["<< d <<"]" << std::endl;

    x = 100;
    y = 50;
    d = film_monitor->get_pixel_sampling_density(x, y);
    std::cerr << "Density at ["<< x <<"] ["<< y <<"] is ["<< d <<"]" << std::endl;

    // To intensity film
    std::shared_ptr<IntensityFilm> intensity_film =
            film_monitor->to_intensity_film();
    intensity_film->write(std::string("/tmp/if.pfm"));
    std::cerr << "Written to /tmp/if.pfm" << std::endl;
}

void test_main3() {
    std::cerr << "Running test main 3" << std::endl;
    std::unique_ptr<IisptScheduleMonitor> schedule_monitor (
                new IisptScheduleMonitor()
                );
    int i = 0;
    while (1) {
        float r = schedule_monitor->get_current_radius();
        if (r < 1.0) {
            exit(0);
        }
        if (i % 50 == 0) {
            std::cerr << "Iteration ["<< i <<"] Radius ["<< r <<"]" << std::endl;
        }
        i++;
    }
}

void test_main2() {
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

void test_main1() {
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

void test_main0() {
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

#endif // IISPT_H
