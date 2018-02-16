:- register_ros_package(knowrob_common).
:- register_ros_package(knowledge_common).
:- register_ros_package(beliefstate).
:- register_ros_package(storage_place).

:- use_module(library('beliefstate')).

:- owl_parser:owl_parse('package://beliefstate/owl/suturo_actions.owl').

:- rosprolog:tf_listener_start.