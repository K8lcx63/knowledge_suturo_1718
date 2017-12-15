:- register_ros_package(knowrob_beliefstate).

:- owl_parser:owl_parse('package://poke_position_calculation/owl/suturo_object.owl').
:- rdf_register_prefix(suturo_object, 'http://knowrob.org/kb/suturo_object.owl#').