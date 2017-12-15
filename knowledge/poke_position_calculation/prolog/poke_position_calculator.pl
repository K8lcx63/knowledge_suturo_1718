:- module(poke_position_calculator,[calculatePokePosition/6]).

:- use_module(library(semweb/rdf_db)).

:- rdf_register_prefix(suturo_object, 'http://knowrob.org/kb/suturo_object.owl#').

:- rdf_meta 
	get_bounding_box(r,-,-,-).

% calculate_poke_position(+X,+Y,+Z,-RX,-RY,-RZ).
%
% calculates to a given point a poke point. The given point is expected to 
% be in the center.
%
% @param X  X coordinate of the center point
% @param Y  Y coordinate of the center point
% @param Z  Z coordinate of the center point
% @param RX X coordinate of the poke point
% @param RY Y coordinate of the poke point
% @param RZ Z coordinate of the poke point
%
% Koordinatensystem: Z nach vorne hinten, Y nach oben unten, X nach links rechts
%
calculatePokePosition(X,Y,Z,RX,RY,RZ) :-
  get_bounding_box(suturo_object:'PfannerIceTea2LPackage', _, Height, Depth), 
  RX is X
  RY is Y+(Height/4),%3/4 der Höhe%
  RZ is Z+Depth.     %1,5fache Tiefe 

% get_bounding_box(+BoundingBoxHandle, -Width, -Height, -Depth).
%
% returns the boundingbox of the given BoundingBoxHandle
%
% @param BoundingBoxHandle The name of the BoundingBoxHandle
% @param Width             The width of the boundingbox
% @param Height            The height of the boundingbox
% @param Depth             The depth of the boundingbox
%
get_bounding_box(BoundingBoxHandle, Width, Height, Depth):-
  class_properties_value(BoundingBoxHandle, knowrob:'boundingBoxSize',BoundingBoxRaw), 
  strip_literal_type(BoundingBoxRaw, BoundingBoxSpaceSpeperated),
  parse_vector(BoundingBoxSpaceSpeperated, BoundingBox),
  nth0(0, BoundingBox, Width),
  nth0(1, BoundingBox, Height),
  nth0(2, BoundingBox, Depth).