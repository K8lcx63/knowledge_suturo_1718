:- register_ros_package(knowrob_common).

:- use_module(kitchen_model_exporter,[getFixedKitchenObjects/20]).

:- owl_parser:owl_parse('../../../iai_maps/iai_semantic_maps/owl/room.owl').

