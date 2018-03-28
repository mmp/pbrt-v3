# A dataset reads into a directory
# The directory can contain several sets, each has 1 directory
#
# Each set contains a train.json file:
# {normalization_intensity, normalization_distance}
# That give the normalization values
#
# Dataset files are named like:
# type_x_y.pfm
# type can be:
# p - path traced ground truth
# d - low quality input
# n - normals
# z - depth
#
# Python dataset representation
# A data item contains keys of the same names:
# {p, d, n, z}
# Each value is a PfmImage object