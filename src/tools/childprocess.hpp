#ifndef CHILD_PROCESS_H
#define CHILD_PROCESS_H

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

namespace pbrt {

class ChildProcess {

private: // ===================================================================

    pid_t child_pid;
    int stdout_pipe[2];
    int stdin_pipe[2];

public: // ====================================================================

    // Constructor ------------------------------------------------------------
    // The array's last element is NULL
    ChildProcess(
            std::string process_path,
            char *const argv[]
            )
    {
        pipe(stdout_pipe);
        pipe(stdin_pipe);
        child_pid = fork();

        if (child_pid == -1) {
            std::cerr << "fork() failed" << std::endl;
            exit(1);
        } else if (child_pid == 0) {
            // Child process

            // Child receives write end of stdout pipe
            dup2(stdout_pipe[1], STDOUT_FILENO);

            // Child receives read end of stdin pipe
            dup2(stdin_pipe[0], STDIN_FILENO);

            close(stdout_pipe[0]);
            close(stdout_pipe[1]);
            close(stdin_pipe[0]);
            close(stdin_pipe[1]);

            execvp(process_path.c_str(), argv);

            std::cerr << "execvp() failed" << std::endl;
            exit(1);
        }

        // Original process
        // Close unused ends
        close(stdout_pipe[1]);
        close(stdin_pipe[0]);
    }

    // ------------------------------------------------------------------------
    // Read char
    char read_char() {
        char buffer[1];
        ssize_t count = read(stdout_pipe[0], buffer, 1);
        if (count <= 0) {
            return '\0';
        } else {
            return buffer[0];
        }
    }

    // ------------------------------------------------------------------------
    // Write char
    void write_char(char c) {
        char buffer[1];
        buffer[0] = c;
        write(stdin_pipe[1], buffer, 1);
    }

    // ------------------------------------------------------------------------
    // Read float32
    // Returns 0 if successful
    // Returns 1 if error
    int read_float32(float* buffer) {
        ssize_t count = read(stdout_pipe[0], buffer, 4);
        if (count != 4) {
            return 1;
        } else {
            return 0;
        }
    }

    // ------------------------------------------------------------------------
    // Write float32
    void write_float32(float val) {
        write(stdin_pipe[1], &val, 4);
    }

};

}

#endif // CHILD_PROCESS_H
