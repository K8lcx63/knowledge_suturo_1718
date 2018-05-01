:- module(knowledge_grasp,
    [
      find_grasp_pose/4,
      find_grasp_individual/2,
      sort_grasp_poses/2
    ]).

:- use_module(library('storage_place'))
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).

:- rdf_register_prefix(suturo_action, 'http://knowrob.org/kb/suturo_action.owl#').
:- rdf_register_prefix(suturo_object, 'http://knowrob.org/kb/suturo_object.owl#').

:-  rdf_meta
    find_grasp_pose(r,-,-,-),
    find_grasp_individual(r,-),
    sort_grasp_poses(r,-).

find_grasp_pose(ObjectClassLabel, Translation, Quaternion, Direction):-
	 find_grasp_individual(ObjectClassLabel, GraspPose),
	 rdf(GraspPose, suturo_object:'graspDirection', LitDirection),
	 strip_literal_type(LitDirection, Direction),
	 transform_data(GraspPose, (Translation, Quaternion)).

find_grasp_individual(ObjectClassLabel, GraspIndividual):-
	 owl_class_properties_value(ObjectClassLabel, suturo_object:'graspableAt', GraspIndividual).

sort_grasp_poses(ObjectClassLabel, SortedGraspList):-
	 findall(X,find_grasp_individual(ObjectClassLabel,X),SortedGraspList).
