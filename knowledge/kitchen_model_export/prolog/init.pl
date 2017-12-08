:- register_ros_package(knowrob_common).
:- register_ros_package(knowrob_household).


:- use_module(kitchen_model_exporter,[getFixedKitchenObjects/20, getFixedKitchenObjects2/4]).

%:- owl_parser:owl_parse('../../../iai_maps/iai_semantic_maps/owl/room.owl').
:- owl_parser:owl_parse('package://iai_semantic_maps/owl/kitchen.owl').