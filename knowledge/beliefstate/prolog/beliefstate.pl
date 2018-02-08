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
    get_rotation(+, -),
    print_beliefstate_intern(+),
    get_all_object_individuals(-),
    get_actions_associated_with_object(r, -),
    get_first_actions_associated_with_object(r, -),
    get_actions_associated_with_object_intern(r, +, -),
    print_actions(+),
    print_action_info(r),
    print_used_gripper(r),
    print_pose(r),
    print_pose_element(+, +),
    print_reference_frame(+).

process_perceive_action(ObjectClass, PoseList, ReferenceFrame):-
    assert_new_individual(knowrob:'SiftPerception', PerceptionActionIndividual),
    assert_new_individual(ObjectClass, ObjectIndividual),
    nth0(0, PoseList, Position),
    nth0(1, PoseList, Quaternion),
    rosprolog:tf_transform_pose(ReferenceFrame, '/map', pose(Position, Quaternion), pose(MapPosition, MapQuaternion)),
    assert_new_pose([MapPosition, MapQuaternion], '/map', PoseIndividual),
    rdf_assert(PerceptionActionIndividual, knowrob:'detectedObject', ObjectIndividual),
    rdf_assert(PerceptionActionIndividual, knowrob:'eventOccursAt', PoseIndividual),
    print_beliefstate.
    
process_grasp_action(ObjectClass, GripperIndividual):-
    assert_new_individual(knowrob:'GraspingSomething', GraspActionIndividual),
    get_latest_action_associated_with_object(ObjectIndividual, LatestActionIndividual),
    rdf_assert(LatestActionIndividual, knowrob:'nextEvent', GraspActionIndividual),
    rdfs_individual_of(ObjectIndividual, ObjectClass),
    rdf_has(LatestActionIndividual, knowrob:'eventOccursAt', LatestPoseIndividual),
    get_pose(LatestPoseIndividual, MapPoseList),
    assert_new_pose_in_gripper_frame(GripperIndividual, MapPoseList, LocalPoseIndividual),
    rdf_assert(GraspActionIndividual, knowrob:'objectActedOn', ObjectIndividual),
    rdf_assert(GraspActionIndividual, knowrob:'deviceUsed', GripperIndividual),
    rdf_assert(GraspActionIndividual, knowrob:'eventOccursAt', LocalPoseIndividual),
    print_beliefstate.
    
process_drop_action(GripperIndividual):-
    assert_new_individual(knowrob:'RealisingGraspOfSomething', DropActionIndividual),
    object_attached_to_gripper(GripperIndividual, ObjectIndividual),
    get_latest_action_associated_with_object(ObjectIndividual, GraspActionIndividual),
    rdf_assert(GraspActionIndividual, knowrob:'nextEvent', DropActionIndividual),
    rdf_has(GraspActionIndividual, knowrob:'eventOccursAt', LatestPoseIndividual),
    get_pose(LatestPoseIndividual, LocalPoseList),
    assert_new_pose_from_gripper_frame(GripperIndividual, LocalPoseList, GlobalPoseIndividual),
    rdf_assert(DropActionIndividual, knowrob:'objectActedOn', ObjectIndividual),
    rdf_assert(DropActionIndividual, knowrob:'deviceUsed', GripperIndividual),
    rdf_assert(DropActionIndividual, knowrob:'eventOccursAt', GlobalPoseIndividual),
    print_beliefstate.

object_attached_to_gripper(GripperIndividual, ObjectIndividual):-
    rdfs_individual_of(GraspActionIndividual, knowrob:'GraspingSomething'),
    \+(rdf_has(GraspActionIndividual, knowrob:'nextEvent', _)),
    rdf_has(GraspActionIndividual, knowrob:'deviceUsed', GripperIndividual),
    rdf_has(GraspActionIndividual, knowrob:'objectActedOn', ObjectIndividual).

get_latest_action_associated_with_object(ObjectIndividual, ActionIndvidual):-
    rdfs_individual_of(ActionIndvidual, knowrob:'Event'),
    rdf_has(ActionIndvidual, knowrob:'objectActedOn', ObjectIndividual),
    \+(rdf_has(ActionIndvidual, knowrob:'nextEvent', _)).

get_latest_object_pose(ObjectIndividual, PoseList):- 
    get_latest_action_associated_with_object(ObjectIndividual, ActionIndvidual),
    rdf_has(ActionIndvidual, knowrob:'eventOccursAt', PoseIndividual),
    get_pose(PoseIndividual, PoseList).

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
    (rdf_equal(GripperIndividual, suturo_action:'left_gripper') ->
        rosprolog:tf_transform_pose('/map', '/l_gripper_led_frame', pose(Position, Quaternion), pose(LocalGripperPosition, LocalGripperQuaternion)),
        assert_new_pose([LocalGripperPosition, LocalGripperQuaternion], 'l_gripper_led_frame', LocalPoseIndividual)
        ;
        rosprolog:tf_transform_pose('/map', '/l_gripper_led_frame', pose(Position, Quaternion), pose(LocalGripperPosition, LocalGripperQuaternion)),
        assert_new_pose([LocalGripperPosition, LocalGripperQuaternion], 'r_gripper_led_frame', LocalPoseIndividual)).

assert_new_pose_from_gripper_frame(GripperIndividual, LocalPoseList, GlobalPoseIndividual):-
    nth0(0, LocalPoseList, Position),
    nth0(1, LocalPoseList, Quaternion),
    (rdf_equal(GripperIndividual, suturo_action:'left_gripper') ->
        rosprolog:tf_transform_pose('/l_gripper_led_frame', '/map', pose(Position, Quaternion), pose(GlobalPosition, GlobalQuaternion)),
        assert_new_pose([GlobalPosition, GlobalQuaternion], '/map', GlobalPoseIndividual)
        ;
        rosprolog:tf_transform_pose('/r_gripper_led_frame', '/map', pose(Position, Quaternion), pose(GlobalPosition, GlobalQuaternion)),
        assert_new_pose([GlobalPosition, GlobalQuaternion], '/map', GlobalPoseIndividual)).

get_pose(PoseIndividual, PoseList):-
    transform_data(PoseIndividual, (Translation, Rotation)),
    append([Translation], [Rotation], PoseList).

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

get_reference_frame(PoseIndividual, ReferenceFrame):-
    rdf_has(PoseIndividual, suturo_action:'referenceFrame', FRaw),
    strip_literal_type(FRaw, ReferenceFrame).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%

print_beliefstate:-
    get_all_object_individuals(ObjectIndividualList),
    print_beliefstate_intern(ObjectIndividualList).

print_beliefstate_intern([]).

print_beliefstate_intern([H|T]):-
    rosprolog:ros_info(H),
    get_actions_associated_with_object(H, ActionIndvidualList),
    print_actions(ActionIndvidualList),
    print_beliefstate_intern(T).

get_all_object_individuals(ObjectIndividualList):-
    findall(Temp, 
           (rdfs_individual_of(Temp, knowrob:'DrinkOrFood'); 
            rdfs_individual_of(Temp, knowrob:'FoodVessel')), 
            ObjectIndividualList).

get_actions_associated_with_object(ObjectIndividual, ActionIndvidualList):-
    get_first_actions_associated_with_object(ObjectIndividual, FirstActionIndividual),
    get_actions_associated_with_object_intern(FirstActionIndividual, [FirstActionIndividual], ActionIndvidualList).

get_first_actions_associated_with_object(ObjectIndividual, ActionIndvidual):-
    rdfs_individual_of(ActionIndvidual, knowrob:'SiftPerception'),
    rdf_has(ActionIndvidual, knowrob:'objectActedOn', ObjectIndividual).

get_actions_associated_with_object_intern(CurrentActionIndividual, TempActionIndividualList, ActionIndvidualList):-
    \+(rdf_has(CurrentActionIndividual, knowrob:'nextEvent', _)),
    append(TempActionIndividualList, [], ActionIndvidualList).

get_actions_associated_with_object_intern(CurrentActionIndividual, TempActionIndividualList, ActionIndvidualList):-
    rdf_has(CurrentActionIndividual, knowrob:'nextEvent', NextActionIndividual),
    append(TempActionIndividualList, [NextActionIndividual], NewActionIndividualList),
    get_actions_associated_with_object_intern(NextActionIndividual, NewActionIndividualList, ActionIndvidualList).

print_actions([]).

print_actions([H|T]):-
    print_action_info(H),
    print_actions(T).

print_action_info(ActionIndvidual):-
    rdfs_individual_of(ActionIndvidual, knowrob:'SiftPerception'),
    rdf_has(ActionIndvidual, knowrob:'eventOccursAt', PoseIndividual),
    rosprolog:ros_info('SiftPerception'),
    print_pose(PoseIndividual).

print_action_info(ActionIndvidual):-
    rdfs_individual_of(ActionIndvidual, knowrob:'GraspingSomething'),
    rdf_has(ActionIndvidual, knowrob:'deviceUsed', GripperIndividual),
    rdf_has(ActionIndvidual, knowrob:'eventOccursAt', PoseIndividual),
    rosprolog:ros_info('GraspingSomething'),
    print_used_gripper(GripperIndividual),
    print_pose(PoseIndividual).

print_action_info(ActionIndvidual):-
    rdfs_individual_of(ActionIndvidual, knowrob:'RealisingGraspOfSomething'),
    rdf_has(ActionIndvidual, knowrob:'deviceUsed', GripperIndividual),
    rdf_has(ActionIndvidual, knowrob:'eventOccursAt', PoseIndividual),
    rosprolog:ros_info('RealisingGraspOfSomething'),
    print_used_gripper(GripperIndividual),
    print_pose(PoseIndividual).

print_used_gripper(GripperIndividual):-
    (rdf_equal(GripperIndividual, suturo_action:'left_gripper') ->
        rosprolog:ros_info('deviceUsed: left gripper')
        ;
        rosprolog:ros_info('deviceUsed: right gripper')).

print_pose(PoseIndividual):-
    get_reference_frame(PoseIndividual, ReferenceFrame),
    get_pose(PoseIndividual, PoseList),
    nth0(0, PoseList, Position),
    nth0(1, PoseList, Quaternion),
    nth0(0, Position, X),
    nth0(1, Position, Y),
    nth0(2, Position, Z),
    nth0(0, Quaternion, XQ),
    nth0(1, Quaternion, YQ),
    nth0(2, Quaternion, ZQ),
    nth0(3, Quaternion, WQ),
    rosprolog:ros_info('Pose:'),
    print_reference_frame(ReferenceFrame),
    print_pose_element('X', X),
    print_pose_element('Y', Y),
    print_pose_element('Z', Z),
    print_pose_element('XQ', XQ),
    print_pose_element('YQ', YQ),
    print_pose_element('ZQ', ZQ),
    print_pose_element('WQ', WQ).

print_pose_element(Identifier, PoseElement):-
    atom_concat(Identifier, ': ', FirstPart),
    atom_concat(FirstPart, PoseElement, Result),
    rosprolog:ros_info(Result).

print_reference_frame(ReferenceFrame):-
    atom_concat('referenceFrame: ', ReferenceFrame, Result),
    rosprolog:ros_info(Result).