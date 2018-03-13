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
var read_line = function(fd, callback) {
    var curr_chars = "";
    var read_char = function() {
        var char_bytelen = 1;
        var buffer = new Buffer(char_bytelen);
        fs.read(fd, buffer, 0, char_bytelen, null, (err, num) => {
            if (num != char_bytelen) {
                callback("Could not read a byte");
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
var read_pixel = function(channels, fd, callback) {
    var pixel_bytelen;
    if (channels == 1) {
        pixel_bytelen = 4;
    } else {
        pixel_bytelen = 12;
    }

    var buffer = new Buffer(pixel_bytelen);
    fs.read(fd, buffer, 0, pixel_bytelen, null, (err, num) => {
        if (pixel_bytelen != num) {
            callback("Could not read ["+ pixel_bytelen +"] bytes. Read ["+ num +"] instead");
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

// Callback: (err, Pfm object)
// path: file path of file to be read
var readPfm = function(file_path, callback) {

    fs.open(file_path, "r", function(status, fd) {
        if (status) {
            console.info(status.message);
            callback("Error when opening file ["+ file_path +"], status ["+ status.message +"]");
            return;
        }
        console.info("Opened binary file: " + file_path);

        var width;
        var height;
        var data;

        var channels;

        async.waterfall([

            // Read the identifier line
            (callback) => {
                read_line(fd, (err, line) => {
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
                read_line(fd, (err, line) => {
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
                read_line(fd, (err, line) => {
                    if (err) {
                        callback(err);
                        return;
                    } else {
                        // Ignore these data
                        callback();
                        return;
                    }
                });
            },

            // Read pixel values
            (callback) => {
                async.times(height, (y_val, next) => {
                    async.times(width, (x_val, next2) => {
                        read_pixel(channels, fd, (err, px) => {
                            if (err) {
                                next2(err);
                                return;
                            }
                            next2(null, px);
                            return;
                        });
                    }, (err, row) => {
                        if (err) {
                            next(err);
                            return;
                        } else {
                            next(null, row);
                            return;
                        }
                    });
                }, (err, rows) => {
                    if (err) {
                        callback(err);
                        return;
                    } else {
                        data = rows;
                        callback();
                        return;
                    }
                });
            }

        ], (err) => {
            if (err) {
                callback(err);
                return;
            } else {
                var the_pfm = new Pfm(width, height, data);
                callback(null, the_pfm);
                return;
            }
        });

    });

};

// Script ---------------------------------------------------------------------

var main = function() {
    if (argv_length() != 1) {
        console.info("Expected 1 argument: input file");
        process.exit();
    }

    var input_file_path = argv_get(0);
    var pfm_image;
    var output_file_path;

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
            var proc = spawn("ristretto", [output_file_path]);
            proc.on("close", function(code, signal) {
                console.info("Ristretto exited with ["+code+"] ["+signal+"]");
                callback();
            });
        },

        // Delete the file
        (callback) => {
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
