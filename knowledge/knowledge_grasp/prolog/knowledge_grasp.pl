:- module(knowledge_grasp,
    [
      find_grasp_pose/4,
      find_grasp_individual/2,
      sort_grasp_poses/3
    ]).

:- use_module(library(pairs)).
:- use_module(library('storage_place')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('knowrob/rdfs')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).

:- rdf_register_prefix(suturo_storage_place, 'http://knowrob.org/kb/suturo_storage_place.owl#').
:- rdf_register_prefix(suturo_action, 'http://knowrob.org/kb/suturo_action.owl#').
:- rdf_register_prefix(suturo_object, 'http://knowrob.org/kb/suturo_object.owl#').

:-  rdf_meta
    find_grasp_pose(r,-,-,-),
    find_grasp_individual(r,-),
    sort_grasp_poses(r,r,-),
    build_grasp_triple(r,-),
    build_pose_list(r,-),
    pair_value(r,-),
    evaluate_pose_by_height(r,-).

find_grasp_pose(ObjectClassLabel, Frame, Translation, Quaternion, Direction):-
	 sort_grasp_poses(ObjectClassLabel, Frame, SortedGraspList),
	 member([GraspPose, Translation, Quaternion], SortedGraspList),
	 rdf(GraspPose, suturo_object:'graspDirection', LitDirection),
	 strip_literal_type(LitDirection, Direction).


find_grasp_individual(ObjectClassLabel, GraspIndividual):-
	 owl_class_properties_value(ObjectClassLabel, suturo_object:'graspableAt', GraspIndividual).

build_grasp_tuple([GraspIndividual, ObjectClassLabel, Frame], [GraspIndividual, ObjectClassLabel, Frame, Translation, Quaternion]):-
	 transform_data(GraspIndividual, (Translation, Quaternion)).

build_pose_list(ObjectClassLabel, Frame, PoseList):-
	 findall([X,ObjectClassLabel, Frame],find_grasp_individual(ObjectClassLabel,X),GraspIndividualList),
	 maplist(build_grasp_tuple, GraspIndividualList, PoseList).

evaluate_pose_by_height([Individual, Class, Frame, Translation, Quaternion], K-[Individual, Translation, Quaternion]):-
	 tf_transform_pose(Frame, "/map", pose(Translation, Quaternion), pose([_, _, MZ], _)),
	 storage_place(Class, [_,_,SZ], _, _),
	 K is MZ - SZ.

pair_value(_-V, V).

sort_grasp_poses(ObjectClassLabel, Frame, SortedGraspList):-
	 build_pose_list(ObjectClassLabel, Frame, PoseList),
	 maplist(evaluate_pose_by_height, PoseList, KeyedPoseList),
	 keysort(KeyedPoseList, SortedKeyedList),
	 maplist(pair_value, SortedKeyedList, SortedGraspList).

pose_test(R):-
	 sort_grasp_poses(suturo_object:'PringlesSalt', "/ja_milch", List),
	 member(R, List).
