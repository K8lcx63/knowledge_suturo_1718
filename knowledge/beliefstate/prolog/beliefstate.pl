:- module(beliefstate,
    [
      process_perceive_action/3,         
      process_grasp_action/2,         
      process_drop_action/1,
      object_attached_to_gripper/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).

:- rdf_register_prefix(suturo_action, 'http://knowrob.org/kb/suturo_action.owl#').
:- rdf_register_prefix(suturo_object, 'http://knowrob.org/kb/suturo_object.owl#').

:-  rdf_meta
    process_perceive_action(r,+, +),
    process_grasp_action(r,r),
    process_drop_action(r),
    object_attached_to_gripper(r,-),
    get_latest_action_associated_with_object(r,-),
    get_latest_object_pose(r, -),
    assert_new_individual(r, -),
    assert_new_pose(+, +, -),
    assert_new_pose_in_gripper_frame(r, +, -),
    assert_new_pose_from_gripper_frame(r, +, -),
    get_pose(r, -),
    get_translation(+, -),
    get_rotation(+, -).

process_perceive_action(ObjectClass, PoseList, ReferenceFrame):-
    assert_new_individual(knowrob:'SiftPerception', PerceptionActionIndividual),
    assert_new_individual(ObjectClass, ObjectIndividual),
    nth0(0, PoseList, Position),
    nth0(1, PoseList, Quaternion),
    rosprolog:tf_transform_pose(ReferenceFrame, '/map', pose(Position, Quaternion), pose(MapPosition, MapQuaternion)),
    assert_new_pose([MapPosition, MapQuaternion], '/map', PoseIndividual),
    rdf_assert(PerceptionActionIndividual, knowrob:'detectedObject', ObjectIndividual),
    rdf_assert(PerceptionActionIndividual, knowrob:'eventOccursAt', PoseIndividual).
    
process_grasp_action(ObjectClass, GripperIndividual):-
    assert_new_individual(knowrob:'GraspSomething', GraspActionIndividual),
    rdfs_individual_of(ObjectIndividual, ObjectClass),
    get_latest_object_pose(ObjectIndividual, MapPoseList),
    assert_new_pose_in_gripper_frame(GripperIndividual, MapPoseList, LocalPoseIndividual),
    rdf_assert(GraspActionIndividual, knowrob:'objectActedOn', ObjectIndividual),
    rdf_assert(GraspActionIndividual, knowrob:'deviceUsed', GripperIndividual),
    rdf_assert(GraspActionIndividual, knowrob:'eventOccursAt', LocalPoseIndividual),
    get_latest_action_associated_with_object(ObjectIndividual, LatestActionIndividual),
    rdf_assert(LatestActionIndividual, knowrob:'nextEvent', GraspActionIndividual).
    
process_drop_action(GripperIndividual):-
    assert_new_individual(knowrob:'RealisingGraspOfSomething', DropActionIndividual),
    object_attached_to_gripper(GripperIndividual, ObjectIndividual),
    get_latest_action_associated_with_object(ObjectIndividual, GraspActionIndividual),
    get_latest_object_pose(ObjectIndividual, LocalPoseIndividual),
    get_pose(LocalPoseIndividual, LocalPoseList),
    assert_new_pose_from_gripper_frame(GripperIndividual, LocalPoseList, GlobalPoseIndividual),
    rdf_assert(DropActionIndividual, knowrob:'objectActedOn', ObjectIndividual),
    rdf_assert(DropActionIndividual, knowrob:'deviceUsed', GripperIndividual),
    rdf_assert(DropActionIndividual, knowrob:'eventOccursAt', GlobalPoseIndividual),
    rdf_assert(GraspActionIndividual, knowrob:'nextEvent', DropActionIndividual).

object_attached_to_gripper(GripperIndividual, ObjectIndividual):-
    rdfs_individual_of(GraspActionIndividual, knowrob:'GraspSomething'),
    \+(rdf_has(GraspActionIndividual, knowrob:'nextEvent', _)),
    rdf_has(GraspActionIndividual, knowrob:'deviceUsed', GripperIndividual),
    rdf_has(GraspActionIndividual, knowrob:'objectActedOn', ObjectIndividual).

get_latest_action_associated_with_object(ObjectIndividual, ActionIndvidual):-
    rdfs_individual_of(ActionIndvidual, knowrob:'Event'),
    rdf_has(ActionIndvidual, knowrob:'objectActedOn', ObjectIndividual),
    \+(rdf_has(ActionIndvidual, knowrob:'nextEvent', _)).

get_latest_object_pose(ObjectIndividual, PoseIndividual):- 
    get_latest_action_associated_with_object(ObjectIndividual, ActionIndvidual),
    rdf_has(ActionIndvidual, knowrob:'eventOccursAt', PoseIndividual).

assert_new_individual(ObjectClass, ObjectIndividual):-
    rdf_instance_from_class(ObjectClass, belief_state, ObjectIndividual),
    rdf_assert(ObjectIndividual, rdf:type, owl:'NamedIndividual', belief_state).

assert_new_pose(PoseList, ReferenceFrame, PoseIndividual):-
    get_translation(PoseList, Translation),
    get_rotation(PoseList, Rotation),
    rdf_instance_from_class(knowrob:'Pose', belief_state, PoseIndividual),
    rdf_assert(PoseIndividual, rdf:type, owl:'NamedIndividual', belief_state),
    rdf_assert(PoseIndividual, knowrob:'translation', literal(type(xsd:string,Translation))),
    rdf_assert(PoseIndividual, knowrob:'quaternion', literal(type(xsd:string,Rotation))),
    rdf_assert(PoseIndividual, suturo_action:'referenceFrame', literal(type(xsd:string,ReferenceFrame))).

assert_new_pose_in_gripper_frame(GripperIndividual, MapPoseList, LocalPoseIndividual):-
    nth0(0, MapPoseList, Position),
    nth0(1, MapPoseList, Quaternion),
    (rdf_equivalent(GripperIndividual, suturo_action:'left_gripper') ->
        rosprolog:tf_transform_pose('/map', '/l_gripper_led_frame', pose(Position, Quaternion), pose(LocalGripperPosition, LocalGripperQuaternion)),
        assert_new_pose([LocalGripperPosition, LocalGripperQuaternion], 'l_gripper_led_frame', LocalPoseIndividual)
        ;
        rosprolog:tf_transform_pose('/map', '/l_gripper_led_frame', pose(Position, Quaternion), pose(LocalGripperPosition, LocalGripperQuaternion)),
        assert_new_pose([LocalGripperPosition, LocalGripperQuaternion], 'r_gripper_led_frame', LocalPoseIndividual)).

assert_new_pose_from_gripper_frame(GripperIndividual, LocalPoseList, GlobalPoseIndividual):-
    nth0(0, LocalPoseList, Position),
    nth0(1, LocalPoseList, Quaternion),
    (rdf_equivalent(GripperIndividual, suturo_action:'left_gripper') ->
        rosprolog:tf_transform_pose('/l_gripper_led_frame', '/map', pose(Position, Quaternion), pose(GlobalPosition, GlobalQuaternion)),
        assert_new_pose([GlobalPosition, GlobalQuaternion], '/map', GlobalPoseIndividual)
        ;
        rosprolog:tf_transform_pose('/r_gripper_led_frame', '/map', pose(Position, Quaternion), pose(GlobalPosition, GlobalQuaternion)),
        assert_new_pose([GlobalPosition, GlobalQuaternion], '/map', GlobalPoseIndividual)).

get_pose(PoseIndividual, PoseList):-
    transform_data(PoseIndividual, (Translation, Rotation)),
    append(Translation, Rotation, PoseList).

get_translation([[X, Y, Z], _], Translation):-
    atom_concat(X, ' ', XTemp),
    atom_concat(Y, ' ', YTemp),
    atom_concat(XTemp, YTemp, Temp),
    atom_concat(Temp, Z, Translation).

get_rotation([_, [X, Y, Z, W]], Rotation):-
    atom_concat(X, ' ', XTemp),
    atom_concat(Y, ' ', YTemp),
    atom_concat(Z, ' ', ZTemp),
    atom_concat(XTemp, YTemp, Temp1),
    atom_concat(ZTemp, W, Temp2),
    atom_concat(Temp1, Temp2, Rotation).