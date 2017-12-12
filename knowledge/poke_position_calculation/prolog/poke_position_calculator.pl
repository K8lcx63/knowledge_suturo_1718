:- module(poke_position_calculator,[calculatePokePosition/6]).

:- use_module(library(semweb/rdf_db)).

:- rdf_register_prefix(suturo_object, 'http://knowrob.org/kb/suturo_object.owl#').

:- rdf_meta 
	get_bounding_box(r,-,-,-).

calculatePokePosition(X,Y,Z,RX,RY,RZ) :-
  get_bounding_box(suturo_object:'PfannerIceTea2LPackage', Width, Height, Depth),
  TX is 1.5*Depth, 
  TZ is Height/4,
  RX is X-TX,%Einfache Tiefe nach hinten drücken	
  RY is Y,   %Mitte der Breite bleibt
  RZ is Z+TZ.%3/4 der Höhe

get_bounding_box(BoundingBoxHandle, Width, Height, Depth):-
  class_properties_value(BoundingBoxHandle, knowrob:'boundingBoxSize',BoundingBoxRaw), 
  strip_literal_type(BoundingBoxRaw, BoundingBoxSpaceSpeperated),
  parse_vector(BoundingBoxSpaceSpeperated, BoundingBox),
  nth0(0, BoundingBox, Width),
  nth0(1, BoundingBox, Height),
  nth0(2, BoundingBox, Depth).