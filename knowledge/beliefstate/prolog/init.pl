:- register_ros_package(knowrob_common).

:- use_module(beliefstate,
    [   
        process_perceive_action/3,         
        process_grasp_action/2,         
        process_drop_action/1,
        object_attached_to_gripper/2
    ]).

:- owl_parser:owl_parse('package://beliefstate/owl/suturo_actions.owl').

:- rosprolog:tf_listener_start.