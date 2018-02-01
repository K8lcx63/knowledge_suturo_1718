:- register_ros_package(knowrob_common).
:- register_ros_package(knowrob_objects).

:- use_module(kitchen_model_exporter,[get_fixed_kitchen_objects/4]).

:- owl_parser:owl_parse('package://knowledge/owl/room.owl').
