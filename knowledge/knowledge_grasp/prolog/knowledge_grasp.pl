:- module(knowledge_grasp,
    [
      find_grasp_pose/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).

:- rdf_register_prefix(suturo_action, 'http://knowrob.org/kb/suturo_action.owl#').
:- rdf_register_prefix(suturo_object, 'http://knowrob.org/kb/suturo_object.owl#').

:-  rdf_meta
    find_grasp_pose(r,-).

find_grasp_pose(ObjectClassLabel, Pose):-
	 owl_class_properties_value(ObjectClassLabel, suturo_object:'graspableAt', GraspPose),
	 rdf_has(GraspPose, suturo_object:'translation', Translation),
	 rdf_has(GraspPose, suturo_object:'quaternion', Quaternion),
	 strip_literal_type(Translation, TranslationStripped),
	 strip_literal_type(Quaternion, QuaternionStripped),
	 atom_string(SAtom1, TranslationStripped),
	 atom_string(SAtom2, QuaternionStripped),
	 atom_concat(SAtom1, ' ', Temp1),
	 atom_concat(Temp1, SAtom2, Pose).
