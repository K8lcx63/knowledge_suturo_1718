:- module(kitchen_model_exporter,[get_fixed_kitchen_objects/4]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- rdf_register_prefix(knowrob, 'http://knowrob.org/kb/knowrob.owl#').

:- rdf_meta 
  get_rotation_matrix_value(r,r,-).

%% get_fixed_kitchen_objects(-ObjectName, -Translation, -Quaternion, -[Width, Height, Depth])).
%
% returns all fixed kitchen objects
%
% @param ObjectName  The name of the kitchen object
% @param Translation The position of the kitchen object
% @param Quaternion  The orientation of the kitchen object
% @param []          The boundingbox of the kitchen object
%
get_fixed_kitchen_objects(ObjectName, Translation, Quaternion, [Width, Height, Depth]) :- 
  rdf_has(Instance,knowrob:'objectActedOn',ObjectName),
  rdfs_individual_of(Instance, knowrob:'SemanticMapPerception'),
  rdf_has(Instance, knowrob:'eventOccursAt',RotationMatrix),
  rotation_matrix_to_list(RotationMatrix, RotationMatrixList),%rotmat_to_list
  matrix(RotationMatrixList, Translation, Quaternion),
  object_dimensions(ObjectName, Depth, Width, Height).

%% rotation_matrix_to_list(+RotationMatrix, -[M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33]).
%
% returns the rotation matrix as list
%
% @param RotationMatrix  Instancename of the rotation matrix
% @param []              Rotation matrix as list
%
rotation_matrix_to_list(RotationMatrix, [M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33]):-
    get_rotation_matrix_value(RotationMatrix, knowrob:'m00', M00), get_rotation_matrix_value(RotationMatrix, knowrob:'m01', M01), get_rotation_matrix_value(RotationMatrix, knowrob:'m02', M02), get_rotation_matrix_value(RotationMatrix, knowrob:'m03', M03),
    get_rotation_matrix_value(RotationMatrix, knowrob:'m10', M10), get_rotation_matrix_value(RotationMatrix, knowrob:'m11', M11), get_rotation_matrix_value(RotationMatrix, knowrob:'m12', M12), get_rotation_matrix_value(RotationMatrix, knowrob:'m13', M13),
    get_rotation_matrix_value(RotationMatrix, knowrob:'m20', M20), get_rotation_matrix_value(RotationMatrix, knowrob:'m21', M21), get_rotation_matrix_value(RotationMatrix, knowrob:'m22', M22), get_rotation_matrix_value(RotationMatrix, knowrob:'m23', M23),
    get_rotation_matrix_value(RotationMatrix, knowrob:'m30', M30), get_rotation_matrix_value(RotationMatrix, knowrob:'m31', M31), get_rotation_matrix_value(RotationMatrix, knowrob:'m32', M32), get_rotation_matrix_value(RotationMatrix, knowrob:'m33', M33).

%% get_rotation_matrix_value(+RotationMatrix, +Property, -Value).
%
% gets value from 4x4 rotation matrix specified by the property.
%
% @param RotationMatrix Instancename of the rotation matrix
% @param Property       The property of the value to be returned
% @param Value          Result as number
%
get_rotation_matrix_value(RotationMatrix, Property, Value):-
  rdf_has(RotationMatrix, Property, Value_R),
  strip_literal_type(Value_R , Value_S),
  atom_number(Value_S, Value).