:- module(kitchen_model_exporter,
    [
      get_fixed_kitchen_objects/3
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- rdf_register_prefix(knowrob, 'http://knowrob.org/kb/knowrob.owl#').
:- rdf_register_prefix(srdl2comp, 'http://knowrob.org/kb/srdl2-comp.owl#').
:- rdf_register_prefix(iaimap, 'http://knowrob.org/kb/IAI-kitchen.owl#').

:- rdf_meta 
 is_iai_kitchen_object(r),
 get_fixed_kitchen_objects(-).

path_to_cad_model(IAIKitchenObjectClass, Path):-
    owl_class_properties_value(IAIKitchenObjectClass, knowrob:'pathToCadModel', Temp),
    strip_literal_type(Temp, Path).

get_frame(ObjectIndividual, Frame):-
  rdf(ObjectIndividual, srdl2comp:'urdfName', literal(TempFrame)),
  atom_concat('iai_kitchen/', TempFrame, Frame).

is_iai_kitchen_object(IAIKitchenObjectClass):-
  rdfs_subclass_of(IAIKitchenObjectClass, knowrob:'IAISink');
  rdfs_subclass_of(IAIKitchenObjectClass, knowrob:'Cupboard');
  rdfs_subclass_of(IAIKitchenObjectClass, knowrob:'IAIThrashDrawer');
  rdfs_subclass_of(IAIKitchenObjectClass, knowrob:'IAIDishWasher');
  rdfs_subclass_of(IAIKitchenObjectClass, knowrob:'IAIDishWasherDoor');
  rdfs_subclass_of(IAIKitchenObjectClass, knowrob:'IAIOvenKnob');
  rdfs_subclass_of(IAIKitchenObjectClass, knowrob:'IAIOven');
  rdfs_subclass_of(IAIKitchenObjectClass, knowrob:'IAIKitchenIslandStove');
  rdfs_subclass_of(IAIKitchenObjectClass, knowrob:'IAIFridge');
  rdfs_subclass_of(IAIKitchenObjectClass, knowrob:'IAIFridgeDoor');
  rdfs_subclass_of(IAIKitchenObjectClass, knowrob:'IAIFridgeDoorHandle');
  rdfs_subclass_of(IAIKitchenObjectClass, knowrob:'IAIDrawer');
  rdfs_subclass_of(IAIKitchenObjectClass, knowrob:'IAIPanel');
  rdfs_subclass_of(IAIKitchenObjectClass, knowrob:'IAIHandle').

get_fixed_kitchen_objects(ObjectIndividual, Path, Frame) :-
  rdfs_individual_of(SemanticMapPerceptionIndividual, knowrob:'SemanticMapPerception'),
  rdf_has(SemanticMapPerceptionIndividual, knowrob:'objectActedOn', ObjectIndividual),
  rdf(ObjectIndividual,rdf:'type', srdl2comp:'UrdfLink'),
  rdf(ObjectIndividual,rdf:'type', Class),
  is_iai_kitchen_object(Class),
  get_frame(ObjectIndividual, Frame),
  path_to_cad_model(Class, Path).