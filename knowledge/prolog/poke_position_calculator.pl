:- module(poke_position_calculator,[calculate_poke_position/6]).

:- use_module(library(semweb/rdf_db)).

:- rdf_register_prefix(suturo_object, 'http://knowrob.org/kb/suturo_object.owl#').

:- rdf_meta 
	get(r,r,r).

calculate_poke_position(X,Y,Z,RX,RY,RZ) :- get_object_sizes(H,W,D),
                                           TX is 1.5*D, 
                                           TZ is H/4,
                                           RX is X-TX,%Einfache Tiefe nach hinten drücken	
                                           RY is Y,   %Mitte der Breite bleibt
                                           RZ is Z+TZ.%3/4 der Höhe

get_object_height(H):- rdf(suturo_object:'Ice_Tea_Package', suturo_object:'bounded', B),
					   get(B, suturo_object:'height', H).

get_object_width(W):- rdf(suturo_object:'Ice_Tea_Package', suturo_object:'bounded', B),
					  get(B, suturo_object:'width', W).

get_object_depth(D):- rdf(suturo_object:'Ice_Tea_Package', suturo_object:'bounded', B),
					  get(B, suturo_object:'depth', D).

get_object_sizes(RH,RW,RD) :- get_object_height(RH),
							  get_object_width(RW),
							  get_object_depth(RD).

get(B, P, R):-
    rdf(B, P, literal(type('http://www.w3.org/2001/XMLSchema#float', T))),
    atom_number(T, R).												