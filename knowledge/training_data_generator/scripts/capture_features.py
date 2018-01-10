#!/usr/bin/env python
import math
import numpy as np
import pickle
import rospy
import rospkg

from training_data_generator.pcl_helper import *
from training_data_generator.training_helper import spawn_model
from training_data_generator.training_helper import delete_model
from training_data_generator.training_helper import initial_setup
from training_data_generator.training_helper import capture_sample
from training_data_generator.features import compute_color_histograms
from training_data_generator.features import compute_normal_histograms
from training_data_generator.srv import GetNormals
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2

def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

def get_training_set_path():
    rospack = rospkg.RosPack()
    packagePath = rospack.get_path('training_data_generator')
    return packagePath + '/data/training_set.sav'

if __name__ == '__main__':
    rospy.init_node('capture_node')

    models = ['beer', 'pringles_paprika']#,
       #'ketchup',
       #'koelln_muesli_honig_knusper_nuss',
       #'weide_milch_small']

    # Disable gravity and delete the ground plane
    initial_setup()
    spawn_model('pringles_paprika')
    labeled_features = []

    # for model_name in models:
    #     spawn_model(model_name)

    #     for i in range(30):
    #     	yaw = i*((2*math.pi)/30)
    #     	capture_sample(yaw)
    #     	#rospy.sleep(2.)
    #         # make five attempts to get a valid a point cloud then give up
    #         # sample_was_good = False
    #         # try_count = 0
    #         # while not sample_was_good and try_count < 5:
    #         #     sample_cloud = capture_sample()
    #         #     sample_cloud_arr = ros_to_pcl(sample_cloud).to_array()

    #         #     # Check for invalid clouds.
    #         #     if sample_cloud_arr.shape[0] == 0:
    #         #         print('Invalid cloud detected')
    #         #         try_count += 1
    #         #     else:
    #         #         sample_was_good = True

    #         # # Extract histogram features
    #         # chists = compute_color_histograms(sample_cloud, using_hsv=True)
    #         # normals = get_normals(sample_cloud)
    #         # nhists = compute_normal_histograms(normals)
    #         # feature = np.concatenate((chists, nhists))
    #         # labeled_features.append([feature, model_name])

    #     delete_model()

rospy.logerr("%s path", get_training_set_path())
pickle.dump(labeled_features, open(get_training_set_path(), 'wb'))