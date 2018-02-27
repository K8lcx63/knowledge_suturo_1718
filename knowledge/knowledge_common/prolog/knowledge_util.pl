:- module(knowledge_util,
    [
      get_pose/2,    
      get_translation/2,  
      get_rotation/2,   
      get_reference_frame/2,         
      pose_list_to_atom/2,
      object_list_to_atom/2,
      round_pose_list/2,
      blue_atom/2,
      yellow_atom/2,
      red_atom/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- rdf_register_prefix(suturo_action, 'http://knowrob.org/kb/suturo_action.owl#').

:- rdf_meta
    get_pose(r, -),
    get_reference_frame(r, -).

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

pose_list_to_atom(PoseList, Atom):-
    round_pose_list(PoseList, [Position, Quaternion]),
    atomic_list_concat(Position, ', ', PositionInnerPart),
    atom_concat('[',PositionInnerPart, PositionTemp),
    atom_concat(PositionTemp, ']', PositionAtom),
    atomic_list_concat(Quaternion, ', ', QuaternionInnerPart),
    atom_concat('[',QuaternionInnerPart, QuaternionTemp),
    atom_concat(QuaternionTemp, ']', QuaternionAtom),
    atom_concat('[', PositionAtom, Temp1),
    atom_concat(Temp1, ',', Temp2),
    atom_concat(Temp2, QuaternionAtom, Temp3),
    atom_concat(Temp3, ']', Atom).

object_list_to_atom(ObjectList, Atom):-
    object_list_to_atom_intern(ObjectList, '', Atom).

object_list_to_atom_intern([], CurrentAtom, Atom):-
    atom_concat(CurrentAtom, '', Atom).

object_list_to_atom_intern([E], CurrentAtom, Atom):-
    rdf_split_url(_,SimpleName, E),
    atom_concat(CurrentAtom, SimpleName, Atom).

object_list_to_atom_intern([H|T], CurrentAtom, Atom):-
    rdf_split_url(_,SimpleName, H),
    atom_concat(SimpleName, ', ', Temp),
    atom_concat(CurrentAtom, Temp, TempAtom),
    object_list_to_atom_intern(T, TempAtom, Atom).

round_two_digits(Float, RoundedFloat):-
    Temp1 is Float*100,
    Temp2 is round(Temp1),
    RoundedFloat is Temp2/100.

round_pose_list([Position, Quaternion], [RoundedPosition, RoundedQuaternion]):-
    apply:maplist(knowledge_util:round_two_digits, Position, RoundedPosition),
    apply:maplist(knowledge_util:round_two_digits, Quaternion, RoundedQuaternion).

blue_atom(Atom, BlueAtom):-
    atom_concat('\033[1;34m', Atom, Temp),
    atom_concat(Temp, '\033[0m\n', BlueAtom).

yellow_atom(Atom, YellowAtom):-
    atom_concat('\033[1;33m', Atom, Temp),
    atom_concat(Temp, '\033[0m\n', YellowAtom).

red_atom(Atom, RedAtom):-
    atom_concat('\033[1;31m', Atom, Temp),
    atom_concat(Temp, '\033[0m\n', RedAtom).