:- module(knowledge_grasp,
    [
      find_grasp_pose/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('beliefstate')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).

:- rdf_register_prefix(suturo_action, 'http://knowrob.org/kb/suturo_action.owl#').
:- rdf_register_prefix(suturo_object, 'http://knowrob.org/kb/suturo_object.owl#').

find_grasp_pose(Object_Label, [[X1,Y1,Z1],[X2,Y2,Z2,W]]):-
	 rdf_has(Object_Label, rdf:type, Object_Class),
	 rdf_has(Object_Class, suturo_object:graspableAt, Grasp_Pose),
	 rdf_has(Grasp_Pose, suturo_object:translation, Translation),
	 rdf_has(Grasp_Pose, suturo_object:quaternion, Quaternion),
	 atom_concat(Translation, Quaternion, [[X1,Y1,Z1],[X2,Y2,Z2,W]]).