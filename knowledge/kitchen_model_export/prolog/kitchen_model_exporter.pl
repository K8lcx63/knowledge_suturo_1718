:- module(kitchen_model_exporter,[getFixedKitchenObjects/4, getFixedKitchenObjects2/4]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- rdf_register_prefix(knowrob, 'http://knowrob.org/kb/knowrob.owl#').
:- rdf_register_prefix(iai-map, 'http://knowrob.org/kb/IAI-kitchen.owl#').

:- rdf_meta 
  getTransform(r,-,-),
  getMeshPath(r,-),
  get_rotation_matrix_value(r,r,-).

getFixedKitchenObjects(Object, Translation, Quaternion, [Width, Height, Depth]) :- 
  rdf_has(A,knowrob:'objectActedOn',Object),
  rdfs_individual_of(A, knowrob:'SemanticMapPerception'),
  rdf_has(A, knowrob:'eventOccursAt',RotationMatrix),
  rotation_matrix_to_list(RotationMatrix, RotationMatrixList),
  matrix_rotation(RotationMatrixList, Quaternion),
  matrix_translation(RotationMatrixList, Translation),
  object_dimensions(Object, Depth, Width, Height).
  
rotation_matrix_to_list(RotationMatrix, [M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33]):-
    get_rotation_matrix_value(RotationMatrix, knowrob:'m00', M00), get_rotation_matrix_value(RotationMatrix, knowrob:'m01', M01), get_rotation_matrix_value(RotationMatrix, knowrob:'m02', M02), get_rotation_matrix_value(RotationMatrix, knowrob:'m03', M03),
    get_rotation_matrix_value(RotationMatrix, knowrob:'m10', M10), get_rotation_matrix_value(RotationMatrix, knowrob:'m11', M11), get_rotation_matrix_value(RotationMatrix, knowrob:'m12', M12), get_rotation_matrix_value(RotationMatrix, knowrob:'m13', M13),
    get_rotation_matrix_value(RotationMatrix, knowrob:'m20', M20), get_rotation_matrix_value(RotationMatrix, knowrob:'m21', M21), get_rotation_matrix_value(RotationMatrix, knowrob:'m22', M22), get_rotation_matrix_value(RotationMatrix, knowrob:'m23', M23),
    get_rotation_matrix_value(RotationMatrix, knowrob:'m30', M30), get_rotation_matrix_value(RotationMatrix, knowrob:'m31', M31), get_rotation_matrix_value(RotationMatrix, knowrob:'m32', M32), get_rotation_matrix_value(RotationMatrix, knowrob:'m33', M33).

get_rotation_matrix_value(RotationMatrix, Property, Value):-
  rdf_has(RotationMatrix, Property, Value_R),
  strip_literal_type(Value_R , Value_S),
  atom_number(Value_S, Value).
















  
getFixedKitchenObjects2(ObjectName, Translation, Quaternion, MeshPath):-
  rdfs_individual_of(Object, knowrob:'SemanticMapPerception'),
  rdf_has(Object, knowrob:'objectActedOn',ObjectName),
  rdf_has(Object, knowrob:'eventOccursAt', Transformation),
  getTransform(Transformation, Translation, Quaternion),
  getMeshPath(ObjectName, MeshPath).


getTransform(TransformationHandle, Translation, Quaternion):-
  transform_data(TransformationHandle, (Translation, Quaternion)).

getMeshPath(BoundingBoxHandle, MeshPath):-
  rdf(BoundingBoxHandle, rdf:'type', Class),
  get_model_path(Class, MeshPath).