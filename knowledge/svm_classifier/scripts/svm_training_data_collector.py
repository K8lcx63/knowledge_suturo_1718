#!/usr/bin/env python

import rospy
import rospkg
import pickle

from knowledge_msgs.msg import TrainingData

def save_training_set():
    rospack = rospkg.RosPack()
    packagePath = rospack.get_path('svm_classifier')
    fullPath = packagePath + '/data/training_set.sav'
    pickle.dump(labeled_features, open(fullPath, 'wb'))

    print('Save training set to disk')

def collect_training_data(training_data_msg):
    labeled_features.append([training_data_msg.features, training_data_msg.label])
    print('Received message with label: ' + training_data_msg.label)

if __name__ == '__main__':
    rospy.init_node('svm_classifier')
    rospy.on_shutdown(save_training_set)

    labeled_features = []
    subscriber = rospy.Subscriber('/feature_extractor/cloud_features' , TrainingData, collect_training_data)
    print('Collecting data...')
    
    rospy.spin()