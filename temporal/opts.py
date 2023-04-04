import configargparse

def get_tdom_options(delayed_parse = False):
    parser = configargparse.ArgumentParser()
    parser.add_argument('--config', is_config_file = True, help='Config file path')
    parser.add_argument("--input_path",    default = "", required = True, help = "Input simulation data folder", type = str)
    parser.add_argument("--input_name",    default = "foam", help = "Input simulation data name", type = str)
    parser.add_argument("--num_transient", default = 500, help = "Number of transient image", type = int)
    
    parser.add_argument("--time_length",   default = -1.0, help = "Temporal duration", type = float)
    parser.add_argument("--sol",           default = 1.0, help = "Speed of light", type = float)
    parser.add_argument("--qnt",           default = 0.99, help = "Normalizing quantile", type = float)
    parser.add_argument("--window_mode",   default = "diag_side_mean", choices=['diag_tri', 'diag_side_mean', 'whole'], help = "Window cropping mode", type = str)

    if delayed_parse:
        return parser
    return parser.parse_args()