:- register_ros_package(knowrob_common).

:- use_module(poke_position_calculator,[calculate_poke_position_left/6, calculate_poke_position_right/6]).

:- owl_parser:owl_parse('package://knowledge/owl/suturo_object.owl').