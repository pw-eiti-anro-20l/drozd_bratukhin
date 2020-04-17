#! /usr/bin/python

import json
from tf.transformations import *

x_axis, y_axis, z_axis = (1, 0, 0), (0, 1, 0), (0, 0, 1)

def convertToFile():

    with open('../yaml/dhparams.json', 'r') as file:

        parameters = json.loads(file.read())

    with open('../yaml/urdf.yaml', 'w') as file:

        for key in parameters.keys():

            a, alpha, d, theta = parameters[key]
            a=float(a)
            d=float(d)
            alpha=float(alpha)
            theta=float(theta)

            tz = translation_matrix((0, 0, d))
            rz = rotation_matrix(theta, z_axis)
            tx = translation_matrix((a, 0, 0))
            rx = rotation_matrix(alpha, x_axis)

            matrix = concatenate_matrices(tz, rz, tx, rx)

            rpy = euler_from_matrix(matrix)
            xyz = translation_from_matrix(matrix)

            file.write(key + ":\n")
            file.write("  j_xyz: {} {} {}\n".format(*xyz))
            file.write("  j_rpy: {} {} {}\n".format(*rpy))
            file.write("  l_xyz: {} 0 0\n".format(xyz[0] / 2))
            file.write("  l_rpy: 0 0 0\n")
            file.write("  l_len: {}\n".format(a))
    with open('../yaml/sizeparams.yaml', 'w') as file:
        avrga = 0
        for key in parameters.keys():
            a, d, alpha, theta = parameters[key]
            a=float(a)
            avrga += a
            file.write('link'+key+'_height: {}\n'.format(a))
        avrga = avrga / 3
        file.write('base_width: {}\n'.format(avrga/2))
        file.write('base_height: {}\n'.format(avrga))
        file.write('linki1_width: {}\n'.format(avrga/6))
        file.write('linki2_width: {}\n'.format(avrga/6))
        file.write('linki3_width: {}\n'.format(avrga/6))
if __name__ == '__main__':

    convertToFile()