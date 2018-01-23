#!/usr/bin/env python

import rospy
import rospkg
import pickle
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
from knowledge_msgs.srv import Classify

def classify(req):
    prediction = clf.predict(scaler.transform(req.features))
    label = encoder.inverse_transform(prediction)[0]
    return ClassifyResponse(label)
  
if __name__ == '__main__':
    rospy.init_node('svm_classifier')

    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    service = rospy.Service('/svm_classifier/classify_service', Classify, classify)

    rospy.spin()