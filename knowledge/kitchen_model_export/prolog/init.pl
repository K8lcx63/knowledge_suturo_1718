:- register_ros_package(knowrob_common).
:- register_ros_package(kitchen_model_export).

:- use_module(library('kitchen_model_exporter')).

%:- owl_parser:owl_parse('package://kitchen_model_export/owl/room.owl').
:- owl_parser:owl_parse('package://iai_semantic_maps/owl/kitchen.owl').
