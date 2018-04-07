#! /usr/bin/env node

var fs = require("fs");
var async = require("async");
var randomstring = require("randomstring");
var path = require("path");
const { spawn } = require('child_process');

var BURN_PERCENTAGE = 0.05;

// Command line arguments -----------------------------------------------------

var argv = process.argv;

var argv_get = function(idx) {
    var real_idx = idx + 2;
    if (real_idx < argv.length) {
        return argv[real_idx];
    } else {
        return null;
    }
};

var argv_length = function() {
    return argv.length - 2;
};

// Numeric utilities ----------------------------------------------------------

var clamp_and_round_8b = function(v) {
    if (v < 0) {
        v = 0;
    }
    if (v > 255) {
        v = 255;
    }
    return Math.round(v);
};

// Class BufferedReader -------------------------------------------------------

function BufferedReader(file_path) {

    var self = this;

    // Callback: function(err)
    self.open = function(callback) {

        fs.open(file_path, "r", function(status, fd) {

            if (status) {
                console.info(status.message);
                callback("Error when opening file ["+file_path+"], status ["+status.message+"]");
                return;
            }
            console.info("Opened binary file: " + file_path);

            // If the buffer is a valid object, the start of the buffer
            // is the next byte to be retrieved
            // If the buffer is null, the buffer is currently empty and we
            // need to read directly from the file
            var BUFFER_SIZE = 100000; // 100 KBytes
            var buffer = null;
            var eof_reached = false;

            // Callback: function(err, buffer)
            self.read_bytes = function(num_bytes, callback) {

                var result_buffer = null;

                async.waterfall([

                    // Read from file if necessary
                    function(callback) {
                        // If buffer is empty, or buffer doesn't have enough bytes, read
                        // from file in chunks of buffer size
                        // Stop reading if EOF is reached
                        var populate_buffer = function(callback) {
                            if ((!eof_reached) && (buffer == null || buffer.length < num_bytes)) {
                                // populate buffer
                                console.info("++ Populating buffer...");
                                if (buffer == null) {
                                    console.info("buffer is null");
                                } else {
                                    console.info("Buffer size " + buffer.length + " requested size " + num_bytes);
                                }
                                var new_buff = new Buffer(BUFFER_SIZE);
                                console.info("Initiating read");
                                fs.read(fd, new_buff, 0, BUFFER_SIZE, null, (err, num) => {
                                    console.info("Read complete ["+err+"] ["+num+"]");
                                    if (err) {
                                        console.info("Read err " + err);
                                        callback(err);
                                        return;
                                    }

                                    console.info("Read ["+ num +"] bytes");

                                    // Reached end of file
                                    if (num < BUFFER_SIZE) {
                                        console.info("EOF reached");
                                        eof_reached = true;
                                        // Trim read buffer to actual size
                                        new_buff = new_buff.slice(0, num);
                                    }

                                    // Append current buffer to existing buffer
                                    if (buffer == null) {
                                        buffer = new_buff;
                                    } else {
                                        buffer = Buffer.concat([buffer, new_buff]);
                                    }

                                    async.setImmediate(() => {
                                        populate_buffer((err) => {
                                            callback(err);
                                        });
                                    });

                                });
                                console.info("fs.read fired");
                            } else {
                                // no need to populate buffer
                                callback();
                            }
                        };
                        populate_buffer(function(err) {
                            callback(err);
                        });
                    },

                    // If buffer has enough data, copy data to result buffer
                    // Otherwise raise an error
                    function(callback) {
                        if (buffer.length < num_bytes) {
                            callback("Requested more bytes than available in file");
                        } else {
                            result_buffer = new Buffer(num_bytes);
                            buffer.copy(result_buffer, 0, 0, num_bytes); // destination start, source start, source end
                            buffer = buffer.slice(num_bytes, buffer.length);
                            // console.info("read bytes. result_buffer ["+result_buffer.length+"] buffer ["+buffer.length+"]");
                            callback();
                        }
                    }

                ], function(err) {
                    callback(err, result_buffer);
                });

            };

            callback();
        });

    };

};

// Class Pixel ----------------------------------------------------------------

// Floating point values pixel
function Pixel(r, g, b) {
    var self = this;

    self.toString = function() {
        return "["+ r +", "+ g +", "+ b +"]";
    };

    self.get_max = function() {
        return Math.max(Math.max(r, g), b);
    };

    self.get_min = function() {
        return Math.min(Math.min(r, g), b);
    };

    self.get_r = function() {
        return r;
    };

    self.get_g = function() {
        return g;
    };

    self.get_b = function() {
        return b;
    };
};

// Class FixedSet -------------------------------------------------------------

// selector_func: function(array) returns an index to be returned
function FixedSet(set_size, selector_func) {

    var self = this;
    var data = [];
    if (set_size < 1) {
        set_size = 1;
    }

    self.add = function(elem) {
        data.push(elem);
        if (data.length > set_size) {
            var removal_index = selector_func(data);
            data.splice(removal_index, 1);
        }
    };

    self.get = function() {
        var ret_index = selector_func(data);
        return data[ret_index];
    };

};

var fixedset_generic_selector = function(data, criteria_func) {
    var t_index = null;
    var t_value = null;
    for (var i = 0; i < data.length; i++) {
        var a_data = data[i];
        if (t_index == null || criteria_func(a_data, t_value)) {
            t_index = i;
            t_value = a_data;
        }
    }
    return t_index;
};

var fixedset_min_selector = function(data) {
    return fixedset_generic_selector(data, (new_data, min_value) => {
        return new_data < min_value;
    });
};

var fixedset_max_selector = function(data) {
    return fixedset_generic_selector(data, (new_data, max_value) => {
        return new_data > max_value;
    });
};

// Class Pfm ------------------------------------------------------------------

function Pfm(width, height, data) {
    var self = this;

    console.info("Effective height is " + data.length);
    for (var y = 0; y < height; y++) {
        var a_width = data[y].length;
        if (a_width != width) {
            console.info("Found a differing width. Expected ["+ width +"] Actual ["+ a_width +"] At y ["+ y +"]");
        }
    }
    console.info("Width is " + width);

    self.getPixel = function(x, y) {
        var row = data[y];
        return row[x];
    };

    self.setPixel = function(x, y, new_pixel) {
        var row = data[y];
        row[x] = new_pixel;
    };

    // Callback: (error)
    self.write_to_ppm = function(out_filename, callback) {

        // Compute number of pixels, and burn/black numbers of pixels
        var num_pixels = width * height;
        var burn_pixels = num_pixels * BURN_PERCENTAGE;

        // Find minimum and maximum threshold values
        var min_set = new FixedSet(burn_pixels, fixedset_max_selector);
        var max_set = new FixedSet(burn_pixels, fixedset_min_selector);
        for (var y = 0; y < height; y++) {
            for (var x = 0; x < width; x++) {
                var a_pixel = self.getPixel(x, y);
                var a_max = a_pixel.get_max();
                var a_min = a_pixel.get_min();
                min_set.add(a_min);
                max_set.add(a_max);
            }
        }
        var val_min = min_set.get();
        var val_max = max_set.get();
        console.info("min ["+val_min+"] max ["+val_max+"]");

        var offset = -val_min;
        var multiplier = 255 / (val_max - val_min);
        var transform_val = function(v) {
            var new_val = v + offset;
            new_val *= multiplier;
            return clamp_and_round_8b(new_val);
        };

        // Open file for writing
        fs.open(out_filename, "w", function(status, fd) {
            if (status) {
                console.info(status.message);
                callback("Error when opening file ["+ out_filename +"], status ["+ status.message +"]");
                return;
            }
            console.info("Opened for writing " + out_filename);

            async.waterfall([

                // Write identifier line
                (callback) => {
                    var buffer = Buffer.from("P6\n");
                    write_string(fd, "P6\n", (err) => {
                        if (err) {
                            callback(err);
                        } else {
                            callback();
                        }
                    });
                },

                // Write width and height
                (callback) => {
                    var line = width + " " + height + "\n";
                    write_string(fd, line, (err) => {
                        if (err) {
                            callback(err);
                        } else {
                            callback();
                        }
                    });
                },

                // Write maximum color value
                (callback) => {
                    var line = "255\n";
                    write_string(fd, line, (err) => {
                        if (err) {
                            callback(err);
                        } else {
                            callback();
                        }
                    });
                },

                // Write pixels
                (callback) => {
                    async.times(height, (y_val, next_y) => {
                        async.times(width, (x_val, next_x) => {
                            var a_pixel = self.getPixel(x_val, y_val);
                            var pix_vals = [];
                            pix_vals.push(transform_val(a_pixel.get_r()));
                            pix_vals.push(transform_val(a_pixel.get_g()));
                            pix_vals.push(transform_val(a_pixel.get_b()));
                            write_pixel(pix_vals, fd, (err) => {
                                next_x(err);
                            });
                        }, (err) => {
                            next_y(err);
                        })
                    }, (err) => {
                        callback(err);
                    });
                },

                // Close the file
                (callback) => {
                    fs.close(fd, (err) => {
                        callback(err);
                    });
                }

            ], (err) => {
                callback(err);
            });
        });

    };

};

// Utilities ------------------------------------------------------------------

// Callback: (err, line)
// line does not contain the line ending
// fd: file descriptor
var read_line = function(buffered_file_reader, callback) {
    var curr_chars = "";
    var read_char = function() {
        var char_bytelen = 1;

        buffered_file_reader.read_bytes(char_bytelen, function(err, buffer) {
            if (err) {
                callback(err);
                return;
            }

            var curr_char = buffer.toString('utf8');
            if (curr_char == '\n') {
                callback(null, curr_chars);
            } else {
                curr_chars += curr_char;
                read_char();
            }
        });

    };
    read_char();
};

// Callback: (err)
var write_buffer = function(fd, bf, callback) {
    fs.write(fd, bf, (err, bytes_written) => {
        if (err) {
            callback(err);
        } else {
            if (bf.length != bytes_written) {
                callback("Error when writing "+ strng +". Could not write ["+ bf.length +"], written ["+ bytes_written +"] bytes");
            } else {
                callback();
            }
        }
    });
};

// Callback: (err)
var write_string = function(fd, strng, callback) {
    var buffer = Buffer.from(strng);
    write_buffer(fd, buffer, (err) => {
        callback(err);
    });
}

// Callback: (err, Pixel)
// fd: file descriptor
var read_pixel = function(channels, buff_file_reader, callback) {
    var pixel_bytelen;
    if (channels == 1) {
        pixel_bytelen = 4;
    } else {
        pixel_bytelen = 12;
    }

    buff_file_reader.read_bytes(pixel_bytelen, function(err, buffer) {
        if (err) {
            callback(err);
            return;
        }

        var r;
        var g;
        var b;

        var r = buffer.readFloatLE(0);
        if (channels == 3) {
            var g = buffer.readFloatLE(4);
            var b = buffer.readFloatLE(8);
        }

        var the_pixel;
        if (channels == 1) {
            the_pixel = new Pixel(r, r, r);
        } else {
            the_pixel = new Pixel(r, g, b);
        }
        callback(null, the_pixel);
        return;
    });

};

// Write an integers pixel (1 byte per channel)
// vals: an array of 3 elements of integers
// Callback: (err)
var write_pixel = function(vals, fd, callback) {
    var buffer = new Buffer(3);
    buffer.writeUInt8(vals[0], 0);
    buffer.writeUInt8(vals[1], 1);
    buffer.writeUInt8(vals[2], 2);
    write_buffer(fd, buffer, (err) => {
        callback(err);
    });
};

// Pfm reader -----------------------------------------------------------------

// Callback: function(err, row)
var read_pixel_row_async = function(buff_file_reader, width, channels, callback) {

    var row = [];
    var x = 0;

    var priv_read_pix = function(callback) {

        // Row complete, exit priv_read_pix
        if (x >= width) {
            callback();
            return;
        }

        // Read a pixel
        read_pixel(channels, buff_file_reader, (err, px) => {
            if (err) {
                callback(err);
                return;
            }

            row.push(px);
            x += 1;
            async.setImmediate(() => {
                priv_read_pix(callback);
            });
        });

    };

    priv_read_pix((err) => {
        callback(err, row);
    });

};

// Callback: (err, Pfm object)
// path: file path of file to be read
var readPfm = function(file_path, callback) {

    var buffered_file_reader = new BufferedReader(file_path);

    var width;
    var height;
    var data;

    var channels;

    async.waterfall([

        // Open the file
        function(callback) {
            console.info("readPfm waterfall: opening reader");
            buffered_file_reader.open(function(err) {
                callback(err);
            });
        },

        // Read the identifier line
        (callback) => {
            console.info("readPfm waterfall: reading identifier line");
            read_line(buffered_file_reader, (err, line) => {
                if (err) {
                    callback(err);
                    return;
                } else {
                    console.info("Read a line ["+ line +"]");
                    if (line == "PF") {
                        channels = 3;
                        callback();
                        return;
                    }
                    if (line == "Pf") {
                        channels = 1;
                        callback();
                        return;
                    }
                    callback("Unrecognized file format. No PFM identifier line");
                    return;
                }
            });
        },

        // Read dimensions line
        (callback) => {
            read_line(buffered_file_reader, (err, line) => {
                if (err) {
                    callback(err);
                    return;
                } else {
                    console.info("Read a line ["+ line +"]");
                    var line_split = line.split(" ");
                    if (line_split.length != 2) {
                        callback("Could not read dimensions");
                        return;
                    }
                    var width_str = line_split[0];
                    var height_str = line_split[1];
                    width = parseInt(width_str);
                    height = parseInt(height_str);
                    console.info("Width ["+ width +"]");
                    console.info("Height ["+ height +"]");
                    callback();
                    return;
                }
            });
        },

        // Read scale factor and endianness
        (callback) => {
            read_line(buffered_file_reader, (err, line) => {
                if (err) {
                    callback(err);
                    return;
                } else {
                    // Ignore these data
                    console.info("Read a line ["+ line +"]");
                    callback();
                    return;
                }
            });
        },

        // Read pixel values
        (callback) => {

            var rows = [];
            var y = 0;

            var priv_read_img = function(callback) {

                // Complete, exit
                if (y >= height) {
                    callback();
                    return;
                }

                // Read row
                read_pixel_row_async(buffered_file_reader, width, channels, (err, a_row) => {
                    if (err) {
                        callback(err);
                        return;
                    }
                    rows.push(a_row);
                    y += 1;
                    async.setImmediate(() => {
                        priv_read_img(callback);
                    });
                });

            };

            priv_read_img((err) => {
                if (err) {
                    callback(err);
                } else {
                    data = rows;
                    callback();
                }
            });

        }

    ], function(err) {
        if (err) {
            callback(err);
        } else {
            var the_pfm = new Pfm(width, height, data);
            callback(null, the_pfm);
        }
    });

};

// Script ---------------------------------------------------------------------

var main = function() {

    var positionals = [];
    var output_file_path = null;
    var batch_mode = false;
    for (var i = 0; i < argv_length(); i++) {
        var an_arg = argv_get(i);
        if (an_arg.startsWith("--")) {
            var s = an_arg.split("=");
            var key = s[0];
            var val = s[1];
            if (key == "--out") {
                output_file_path = val;
                batch_mode = true;
                console.info("Forcing output to " + output_file_path);
            }
        } else {
            positionals.push(an_arg);
        }
    }

    if (positionals.length != 1) {
        console.info("Usage: pfm <input.pfm> [--out=path.ppm]");
        process.exit();
    }

    var input_file_path = positionals[0];
    var pfm_image;

    async.waterfall([

        // Read input PFM
        (callback) => {
            readPfm(input_file_path, (err, res) => {
                if (err) {
                    callback(err);
                } else {
                    console.info("Got the pfm!");
                    pfm_image = res;
                    callback();
                }
            });
        },

        // Generate output filename
        (callback) => {

            if (output_file_path) {
                // Output path was provided on command line
                callback();
                return;
            }

            var MAX_ATTEMPTS = 25;
            var attempt = 0;
            var output_path_valid = false;
            var output_path;

            async.whilst(
                function() {
                    return !output_path_valid && (attempt < MAX_ATTEMPTS);
                },
                function(callback) {
                    attempt += 1;
                    var randstr = "/tmp/" + randomstring.generate(20) + ".ppm";
                    fs.access(randstr, fs.constants.R_OK, (err) => {
                        if (err) {
                            // Does not exist
                            output_path_valid = true;
                            output_path = randstr;
                            callback();
                        } else {
                            // Exists
                            callback();
                        }
                    });
                },
                function(err) {
                    if (err) {
                        callback(err);
                    } else {
                        if (output_path_valid) {
                            output_file_path = output_path;
                            callback();
                        } else {
                            callback("Could not generate output path");
                        }
                    }
                }
            );

        },

        // Write output file
        (callback) => {
            console.info("Writing to " + output_file_path);
            pfm_image.write_to_ppm(output_file_path, (err) => {
                callback(err);
            })
        },

        // Open viewer
        (callback) => {
            if (batch_mode) {
                // Do not open viewer in batch mode
                callback();
                return;
            }

            var proc = spawn("ristretto", [output_file_path]);
            proc.on("close", function(code, signal) {
                console.info("Ristretto exited with ["+code+"] ["+signal+"]");
                callback();
            });
        },

        // Delete the file
        (callback) => {
            if (batch_mode) {
                // Do not delete in batch mode
                callback();
                return;
            }

            fs.unlink(output_file_path, (err) => {
                callback(err);
            });
        }

    ], (err) => {
        if (err) {
            console.info(err);
        } else {
            console.info("Completed.");
        }
    });
}

main();
