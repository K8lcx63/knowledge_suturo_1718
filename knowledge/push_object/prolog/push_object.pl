:- module(push_object,
    [
      calculate_push_pose/5
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#',  [keep(true)]).
:- rdf_register_prefix(suturo_action, 'http://knowrob.org/kb/suturo_action.owl#').
:- rdf_register_prefix(suturo_object, 'http://knowrob.org/kb/suturo_object.owl#').

:- rdf_meta
    calculate_push_pose(r, +, -, -, -),
    object_depth(r,-).

object_depth(ObjectClass, Depth):-
    owl_class_properties_value(ObjectClass, knowrob:'depthOfObject', DepthRaw),
    strip_literal_type(DepthRaw, AtomDepth),
    atom_number(AtomDepth, Depth).

calculate_push_pose(ObjectClass, SourceFrame, X, Y, Z):-
    tf_transform_point(SourceFrame, '/map', [0.0,0.0,0.0], [MX, Y, Z]),
    object_depth(ObjectClass, Depth),
    X is MX - Depth/2.