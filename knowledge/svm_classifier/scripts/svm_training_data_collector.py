#!/usr/bin/env python

import rospy
import numpy as np
import rospkg
import pickle
import csv
import os

def save_training_set():
    fullPath = packagePath + '/data/training_set.sav'
    pickle.dump(labeled_features, open(fullPath, 'wb'))

    print('Save training set to disk')

def get_features(label, color_histogram_file, normals_histogram_file):
    print('process ' + color_histogram_file + ' and ' + normals_histogram_file)

    color_histogram_path = packagePath + '/data/histograms/color_histograms/' + label + '/' + color_histogram_file
    normals_histogram_path = packagePath + '/data/histograms/normals_histograms/' + label + '/' + normals_histogram_file

    color_histogram_reader = csv.reader(open(color_histogram_path))
    color_histogram = np.array(map(int, next(color_histogram_reader)))

    normals_histogram_reader = csv.reader(open(normals_histogram_path))
    normals_histogram = np.array(map(int, next(normals_histogram_reader)))

    return np.concatenate((color_histogram, normals_histogram))

def collect_training_data():
    labels = ['tomato_sauce_oro_di_parma', 'hela_curry_ketchup', 
              'pringles_paprika', 'pringles_salt', 
              'ja_milch', 'koelln_muesli_knusper_honig_nuss', 'kelloggs_toppas_mini',
              'cup_eco_orange', 'edeka_red_bowl', 'sigg_bottle']

    for label in labels:
        color_histogram_files = os.listdir(packagePath + '/data/histograms/color_histograms/' + label)
        normals_histogram_files = os.listdir(packagePath + '/data/histograms/normals_histograms/' + label)
        for i in range(len(color_histogram_files)):
            features = get_features(label, color_histogram_files[i], normals_histogram_files[i])
            labeled_features.append([features, label])

if __name__ == '__main__':
    rospy.init_node('svm_training_data_collector')

    rospack = rospkg.RosPack()
    packagePath = rospack.get_path('svm_classifier')

    labeled_features = []
    collect_training_data()
    save_training_set()