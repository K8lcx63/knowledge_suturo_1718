#!/usr/bin/env python

import rospy
import numpy as np
import rospkg
import csv
import os
from knowledge_msgs.srv import *

def get_corresponding_normals_histogram_file(color_histogram_file):
    temp = color_histogram_file.split('_')
    file_prefix = temp[0] + '_' + temp[1] 
    normals_histogram_file = file_prefix + '_normal.csv'

    return normals_histogram_file

def get_feature(test_directory, color_histogram_file):
    normals_histogram_file = get_corresponding_normals_histogram_file(color_histogram_file)
    print('process ' + color_histogram_file + ' and ' + normals_histogram_file)

    color_histogram_path =packagePath + '/data/test/' + test_directory + '/color_histograms/' + color_histogram_file
    normals_histogram_path = packagePath + '/data/test/' + test_directory + '/normals_histograms/' + normals_histogram_file

    color_histogram_reader = csv.reader(open(color_histogram_path))
    color_histogram = np.array(map(int, next(color_histogram_reader)))

    normals_histogram_reader = csv.reader(open(normals_histogram_path))
    normals_histogram = np.array(map(int, next(normals_histogram_reader)))

    return np.concatenate((color_histogram, normals_histogram))

def get_features(test_directory, color_histogram_files):
    features = []
    for color_histogram_file in color_histogram_files:
        feature = get_feature(test_directory, color_histogram_file)
        features.append(feature)

    return features

def run_test(test_directory):
    color_histogram_files = os.listdir(packagePath + '/data/test/' + test_directory + '/color_histograms/')
    features = get_features(test_directory, color_histogram_files)
    for feature in features:
        classify_service = rospy.ServiceProxy('/svm_classifier/classify_service', Classify)
        classify_service_response = classify_service(feature.tolist())
        print(classify_service_response.label)

if __name__ == '__main__':
    rospy.init_node('svm_test')

    rospack = rospkg.RosPack()
    packagePath = rospack.get_path('svm_classifier')

    run_test('koelln_toppas_sigg_hela_overlap')