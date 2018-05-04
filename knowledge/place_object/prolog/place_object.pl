:- module(place_object,
    [
      calculate_place_pose/3,
      object_height/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_register_prefix(suturo_action, 'http://knowrob.org/kb/suturo_action.owl#').
:- rdf_register_prefix(suturo_object, 'http://knowrob.org/kb/suturo_object.owl#').

:- rdf_meta
    get_grasp_pose_individual(r, +, -),
    object_height(r,-),
    calculate_place_pose(r, -, -).

get_grasp_pose_individual(ObjectClass, [[X,Y,Z],[QX,QY,QZ,QW]], GraspPoseIndividual):-
    owl_class_properties_value(ObjectClass, suturo_object:'graspableAt', GraspPoseIndividual),
    get_pose(GraspPoseIndividual, [[GX,GY,GZ],[GQX,GQY,GQZ,GQW]]),
    GX =:= X,
    GY =:= Y,
    GZ =:= Z,
    GQX =:= QX,
    GQY =:= QY,
    GQZ =:= QZ,
    GQW =:= QW.

object_height(ObjectClass, Height):-
    owl_class_properties_value(ObjectClass, knowrob:'heightOfObject', HeightRaw),
    strip_literal_type(HeightRaw, AtomHeight),
    atom_number(AtomHeight, Height).

calculate_place_pose(GripperIndividual, Z, [QX, QY, QZ, QW]):-
    object_attached_to_gripper(GripperIndividual, ObjectIndividual),
    rdfs_type_of(ObjectIndividual, ObjectClass),
    get_latest_grasp_pose(ObjectClass, [[GX,GY,GZ],[GQX,GQY,GQZ,GQW]]),
    get_grasp_pose_individual(ObjectClass, [[GX,GY,GZ],[GQX,GQY,GQZ,GQW]], GraspPoseIndividual),
    (rdfs_type_of(GraspPoseIndividual, suturo_object:'TopGrasp') ->
        QX is 0.0,
        QY is 0.707,
        QZ is 0.0,
        QW is 0.707
        ;
        QX is 0.0,
        QY is 0.0,
        QZ is 0.0,
        QW is 1.0
    ),
	object_height(ObjectClass, Height),
    Z is GZ + (Height/2) + 0.85.%Tischhöhe=0.85