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
    assert_new_individual(r, -),
    assert_new_pose(+, -),
    assert_new_pose_in_gripper_frame(r, +, -),
    assert_new_pose_from_gripper_frame(r, +, -).

assert_new_individual(ObjectClass, ObjectIndividual):-
    rdf_instance_from_class(ObjectClass, belief_state, ObjectIndividual),
    rdf_assert(ObjectIndividual, rdf:type, owl:'NamedIndividual', belief_state).

assert_new_pose(PoseList, PoseIndividual):-
    get_translation(PoseList, Translation),
    get_rotation(PoseList, Rotation),
    rdf_instance_from_class(knowrob:'Pose', belief_state, PoseIndividual),
    rdf_assert(PoseIndividual, rdf:type, owl:'NamedIndividual', belief_state),
    rdf_assert(PoseIndividual, knowrob:'translation', literal(type(xsd:string,Translation))),
    rdf_assert(PoseIndividual, knowrob:'quaternion', literal(type(xsd:string,Rotation))).

assert_new_pose_in_gripper_frame(GripperIndividual, MapPoseList, LocalPoseIndividual):-
    (rdf_equivalent(GripperIndividual, suturo_action:'left_gripper') ->
        transform_pose_prolog(MapPoseList, '/map', '/l_gripper_led_frame', LocalPoseList),
        assert_new_pose(LocalPoseList, 'l_gripper_led_frame', LocalPoseIndividual)
        ;
        transform_pose_prolog(MapPoseList, '/map', '/r_gripper_led_frame', LocalPoseList),
        assert_new_pose(LocalPoseList, 'r_gripper_led_frame', LocalPoseIndividual)).

assert_new_pose_from_gripper_frame(GripperIndividual, LocalPoseList, GlobalPoseIndividual):-
    (rdf_equivalent(GripperIndividual, suturo_action:'left_gripper') ->
        transform_pose_prolog(LocalPoseList, '/l_gripper_led_frame', '/map', GlobalPoseList)
        ;
        transform_pose_prolog(LocalPoseList, '/r_gripper_led_frame', '/map', GlobalPoseList)),
    assert_new_pose(GlobalPoseList, GlobalPoseIndividual).

%assert_new_time_point(ActionIndvidual):-
%    current_time(Now),
%    assert_new_individual(knowrob:'TimePoint', TimePointIndividual),
%    rdf_assert(TimePointIndividual, knowrob:'duration', literal(type(xsd:float,Now))),
%    rdf_assert(ActionIndvidual, knowrob:'startTime', TimePointIndividual).

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

get_pose(PoseIndividual, PoseList):-
    transform_data(PoseIndividual, (Translation, Rotation)),
    append(Translation, Rotation, PoseList).

get_latest_action_associated_with_object(ObjectIndividual, ActionIndvidual):-
    rdfs_subclass_of(ActionIndvidual, knowrob:'Event'),
    rdf_has(ActionIndvidual, knowrob:'objectActedOn', ObjectIndividual),
    \+(rdf_has(ActionIndvidual, knowrob:'nextEvent', _)).

get_latest_object_pose(ObjectIndividual, PoseIndividual):- 
    get_latest_action_associated_with_object(ObjectIndividual, ActionIndvidual),
    rdf_has(ActionIndvidual, knowrob:'eventOccursAt', PoseIndividual).
                                            
object_attached_to_gripper(GripperIndividual, ObjectIndividual):-
    rdfs_subclass_of(GraspActionIndividual, knowrob:'GraspSomething'),
    \+(rdf_has(GraspActionIndividual, knowrob:'nextEvent', _)),
    rdf_has(GraspActionIndividual, knowrob:'deviceUsed', GripperIndividual),
    rdf_has(GraspActionIndividual, knowrob:'objectActedOn', ObjectIndividual).

process_perceive_action(ObjectClass, PoseList, ReferenceFrame):-
    assert_new_individual(knowrob:'SiftPerception', PerceptionActionIndividual),
    assert_new_individual(ObjectClass, ObjectIndividual),
    tfjava_transform_pose(ReferenceFrame, '/map', PoseList, MapPoseList, _),
    assert_new_pose(MapPoseList, PoseIndividual),
    %assert_new_time_point(PerceptionActionIndividual),
    rdf_assert(PerceptionActionIndividual, knowrob:'detectedObject', ObjectIndividual),
    rdf_assert(PerceptionActionIndividual, knowrob:'eventOccursAt', PoseIndividual).
    
process_grasp_action(ObjectClass, GripperIndividual):-
    assert_new_individual(knowrob:'GraspSomething', GraspActionIndividual),
    rdfs_individual_of(ObjectIndividual, ObjectClass),
    get_latest_object_pose(ObjectIndividual, MapPoseList),
    assert_new_pose_in_gripper_frame(GripperIndividual, MapPoseList, LocalPoseIndividual),
    %assert_new_time_point(GraspActionIndividual),
    rdf_assert(GraspActionIndividual, knowrob:'objectActedOn', ObjectIndividual),
    rdf_assert(GraspActionIndividual, knowrob:'deviceUsed', GripperIndividual),
    rdf_assert(GraspActionIndividual, knowrob:'eventOccursAt', LocalPoseIndividual),
    get_latest_action_associated_with_object(ObjectIndividual, LatestActionIndividual),
    rdf_assert(LatestActionIndividual, knowrob:'nextEvent', GraspActionIndividual).
    
process_drop_action(GripperIndividual):-
    assert_new_individual(knowrob:'RealisingGraspOfSomething', DropActionIndividual),
    object_attached_to_gripper(GripperIndividual, ObjectIndividual),
    get_latest_action_associated_with_object(ObjectIndividual, GraspActionIndividual),
    rdf_has(GraspActionIndividual, knowrob:'eventOccursAt', GraspActionPoseIndividual),
    get_pose(GraspActionPoseIndividual, LocalPoseList),
    assert_new_pose_from_gripper_frame(GripperIndividual, LocalPoseList, GlobalPoseIndividual),
    %assert_new_time_point(DropActionIndividual),
    rdf_assert(DropActionIndividual, knowrob:'objectActedOn', ObjectIndividual),
    rdf_assert(DropActionIndividual, knowrob:'deviceUsed', GripperIndividual),
    rdf_assert(DropActionIndividual, knowrob:'eventOccursAt', GlobalPoseIndividual),
    rdf_assert(GraspActionIndividual, knowrob:'nextEvent', DropActionIndividual).