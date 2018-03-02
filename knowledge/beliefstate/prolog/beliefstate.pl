:- module(beliefstate,
    [
      object_exists/1,
      process_perceive_action/3,         
      process_grasp_action/2,         
      process_drop_action/1,
      object_attached_to_gripper/2,
      get_latest_object_pose/2,
      get_objects_on_kitchen_island_counter/1,
      get_two_objects_on_kitchen_island_counter_with_same_storage_place/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).

:- rdf_register_prefix(suturo_action, 'http://knowrob.org/kb/suturo_action.owl#').
:- rdf_register_prefix(suturo_object, 'http://knowrob.org/kb/suturo_object.owl#').

:-  rdf_meta
    object_exists(r),
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
    print_beliefstate_intern(+),
    get_all_object_individuals(-),
    get_first_actions_associated_with_object(r, -),
    get_actions_associated_with_object(r, -),
    get_actions_associated_with_object_intern(r, +, -),
    print_actions(+),
    print_action_info(r),
    get_pose_info(r, -),
    get_used_gripper_info(r, +, -).

object_exists(ObjectClass):-
    rdfs_individual_of(_, ObjectClass).

process_perceive_action(ObjectClass, PoseList, ReferenceFrame):-
    assert_new_individual(knowrob:'SiftPerception', PerceptionActionIndividual),
    (rdfs_individual_of(ObjectIndividual, ObjectClass) ->
        get_latest_action_associated_with_object(ObjectIndividual, LatestActionIndividual),
        rdf_assert(LatestActionIndividual, knowrob:'nextEvent', PerceptionActionIndividual)
        ;
        assert_new_individual(ObjectClass, ObjectIndividual)
    ),!,
    nth0(0, PoseList, Position),
    nth0(1, PoseList, Quaternion),
    tf_transform_pose(ReferenceFrame, '/map', pose(Position, Quaternion), pose(MapPosition, MapQuaternion)),
    assert_new_pose([MapPosition, MapQuaternion], '/map', PoseIndividual),
    rdf_assert(PerceptionActionIndividual, knowrob:'detectedObject', ObjectIndividual),
    rdf_assert(PerceptionActionIndividual, knowrob:'eventOccursAt', PoseIndividual),
    print_beliefstate,!.
    
process_grasp_action(ObjectClass, GripperIndividual):-
    assert_new_individual(knowrob:'GraspingSomething', GraspActionIndividual),
    rdfs_individual_of(ObjectIndividual, ObjectClass),
    get_latest_action_associated_with_object(ObjectIndividual, LatestActionIndividual),
    rdf_assert(LatestActionIndividual, knowrob:'nextEvent', GraspActionIndividual),
    rdf_has(LatestActionIndividual, knowrob:'eventOccursAt', LatestPoseIndividual),
    get_pose(LatestPoseIndividual, MapPoseList),
    assert_new_pose_in_gripper_frame(GripperIndividual, MapPoseList, LocalPoseIndividual),
    rdf_assert(GraspActionIndividual, knowrob:'objectActedOn', ObjectIndividual),
    rdf_assert(GraspActionIndividual, knowrob:'deviceUsed', GripperIndividual),
    rdf_assert(GraspActionIndividual, knowrob:'eventOccursAt', LocalPoseIndividual),
    print_beliefstate,!.
    
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
    print_beliefstate,!.

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
        tf_transform_pose('/map', '/l_gripper_led_frame', pose(Position, Quaternion), pose(LocalGripperPosition, LocalGripperQuaternion)),
        assert_new_pose([LocalGripperPosition, LocalGripperQuaternion], 'l_gripper_led_frame', LocalPoseIndividual)
        ;
        tf_transform_pose('/map', '/l_gripper_led_frame', pose(Position, Quaternion), pose(LocalGripperPosition, LocalGripperQuaternion)),
        assert_new_pose([LocalGripperPosition, LocalGripperQuaternion], 'r_gripper_led_frame', LocalPoseIndividual)).

assert_new_pose_from_gripper_frame(GripperIndividual, LocalPoseList, GlobalPoseIndividual):-
    nth0(0, LocalPoseList, Position),
    nth0(1, LocalPoseList, Quaternion),
    (rdf_equal(GripperIndividual, suturo_action:'left_gripper') ->
        tf_transform_pose('/l_gripper_led_frame', '/map', pose(Position, Quaternion), pose(GlobalPosition, GlobalQuaternion)),
        assert_new_pose([GlobalPosition, GlobalQuaternion], '/map', GlobalPoseIndividual)
        ;
        tf_transform_pose('/r_gripper_led_frame', '/map', pose(Position, Quaternion), pose(GlobalPosition, GlobalQuaternion)),
        assert_new_pose([GlobalPosition, GlobalQuaternion], '/map', GlobalPoseIndividual)).

get_objects_on_kitchen_island_counter(ObjectIndividualList):-
    findall(ObjectIndividual , 
           (rdfs_individual_of(ActionIndvidual, knowrob:'SiftPerception'),
           \+(rdf_has(ActionIndvidual, knowrob:'nextEvent', _)),
           rdf_has(ActionIndvidual, knowrob:'objectActedOn', ObjectIndividual)),
           ObjectIndividualList).

get_two_objects_on_kitchen_island_counter_with_same_storage_place(Object1, Object2):-
    get_objects_on_kitchen_island_counter(ObjectList),
    member(Object1, ObjectList),
    member(Object2, ObjectList),
    \+rdf_equal(Object1, Object2),
    storage_area(Object1, StorageArea1),
    storage_area(Object2, StorageArea2),
    rdf_equal(StorageArea1, StorageArea2).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%

print_beliefstate:-
    ros_info('###################################'),
    get_all_object_individuals(ObjectIndividualList),
    print_beliefstate_intern(ObjectIndividualList),
    get_objects_on_kitchen_island_counter(ObjectList),
    object_list_to_atom(ObjectList, ObjectListAtom),
    atom_concat('Remaining objects: ' , ObjectListAtom, Temp),
    red_atom(Temp, RedObjectListAtom),
    ros_info(RedObjectListAtom).

print_beliefstate_intern([]).

print_beliefstate_intern([H|T]):-
    rdf_split_url(_,SimpleName, H),
    blue_atom(SimpleName, BlueSimpleName),
    ros_info(BlueSimpleName),
    get_actions_associated_with_object(H, ActionIndvidualList),
    print_actions(ActionIndvidualList),
    print_beliefstate_intern(T).

get_all_object_individuals(ObjectIndividualList):-
    findall(Temp, 
           (rdfs_individual_of(Temp, knowrob:'Sauce'); 
            rdfs_individual_of(Temp, knowrob:'Snacks');
            rdfs_individual_of(Temp, knowrob:'BreakfastCereal');
            rdfs_individual_of(Temp, knowrob:'Drink');
            rdfs_individual_of(Temp, knowrob:'FoodVessel')), 
            ObjectIndividualList).

get_first_action_associated_with_object(ObjectIndividual, ActionIndvidual):-
    rdfs_individual_of(ActionIndvidual, knowrob:'Event'),
    rdf_has(ActionIndvidual, knowrob:'objectActedOn', ObjectIndividual),
    \+(rdf_has(_, knowrob:'nextEvent', ActionIndvidual)).

get_actions_associated_with_object(ObjectIndividual, ActionIndvidualList):-
    get_first_action_associated_with_object(ObjectIndividual, FirstActionIndividual),
    get_actions_associated_with_object_intern(FirstActionIndividual, [FirstActionIndividual], ActionIndvidualList).

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
    get_pose_info(ActionIndvidual, PoseInfoAtom),
    (rdfs_individual_of(ActionIndvidual, knowrob:'SiftPerception') ->
        atom_concat('PerceiveAction', ': ', ActionInfoAtom),
        atom_concat(ActionInfoAtom, PoseInfoAtom, WholeInfoAtom)
        ;
        (rdfs_individual_of(ActionIndvidual, knowrob:'GraspingSomething') ->
            atom_concat('GraspAction', ': ', ActionInfoAtom)
            ;
            atom_concat('DropAction', ': ', ActionInfoAtom)
        ),
        rdf_has(ActionIndvidual, knowrob:'deviceUsed', GripperIndividual),
        get_used_gripper_info(GripperIndividual, ActionInfoAtom, CurrentInfoAtom),
        atom_concat(CurrentInfoAtom, PoseInfoAtom, WholeInfoAtom)
    ),
    yellow_atom(WholeInfoAtom, WholeInfoYelloqAtom),
    ros_info(WholeInfoYelloqAtom).

get_pose_info(ActionIndvidual, PoseInfoAtom):-
    rdf_has(ActionIndvidual, knowrob:'eventOccursAt', PoseIndividual),
    get_pose(PoseIndividual, PoseList),
    get_reference_frame(PoseIndividual, ReferenceFrame),
    atom_concat(ReferenceFrame, ', ', TempReferenceFrame),
    pose_list_to_atom(PoseList, PoseListAtom),
    atom_concat(TempReferenceFrame, PoseListAtom, PoseInfoAtom).

get_used_gripper_info(GripperIndividual, CurrentInfoAtom, ModifiedInfoAtom):-
    (rdf_equal(GripperIndividual, suturo_action:'left_gripper') ->
        atom_concat(CurrentInfoAtom, 'leftGripper, ', ModifiedInfoAtom)
        ;
        atom_concat(CurrentInfoAtom, 'rightGripper, ', ModifiedInfoAtom)
    ).