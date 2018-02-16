:- register_ros_package(knowrob_common).
:- register_ros_package(storage_place).

:- use_module(library('storage_place')).

:- owl_parser:owl_parse('package://storage_place/owl/suturo_storage_place.owl').