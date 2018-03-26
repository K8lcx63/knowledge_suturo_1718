:- module(place_object,
    [
      calculate_place_z_position/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_register_prefix(suturo_action, 'http://knowrob.org/kb/suturo_action.owl#').
:- rdf_register_prefix(suturo_object, 'http://knowrob.org/kb/suturo_object.owl#').

:- rdf_meta
    object_height(r,-),
    calculate_place_z_position(r, -).

object_height(ObjectClass, Height):-
    owl_class_properties_value(ObjectClass, knowrob:'heightOfObject', HeightRaw),
    strip_literal_type(HeightRaw, Height).

calculate_place_z_position(GripperIndividual, Z):-
    object_attached_to_gripper(GripperIndividual, ObjectIndividual),
    get_latest_grasp_pose(ObjectIndividual, [[_,_,GraspPoseZ], _]),
    rdfs_individual_of(ObjectIndividual, ObjectClass),
	object_height(ObjectClass, Height),
    Z is GraspPoseZ + (Height/2) + 0.85. %Tischhöhe