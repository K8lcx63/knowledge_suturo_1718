#!/usr/bin/env python

import rospy
import rospkg
import pickle
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
from knowledge_msgs.srv import *

def classify(req):
    temp = map(float, req.features)
    np_array = np.array(temp)
    prediction = clf.predict(scaler.transform(np_array.reshape(1,-1)))
    label = encoder.inverse_transform(prediction)[0]
    return ClassifyResponse(label)
  
if __name__ == '__main__':
    rospy.init_node('svm_classifier')
    
    rospack = rospkg.RosPack()
    packagePath = rospack.get_path('svm_classifier')
    fullPath = packagePath + '/data/svm_model.sav'
    model = pickle.load(open(fullPath, 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    service = rospy.Service('/svm_classifier/classify_service', Classify, classify)

    rospy.spin()