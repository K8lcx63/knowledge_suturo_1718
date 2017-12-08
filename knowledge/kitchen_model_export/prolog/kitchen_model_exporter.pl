:- module(kitchen_model_exporter,[getFixedKitchenObjects/20, getFixedKitchenObjects2/5]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

%% Register owl namespaces
%:- rdf_db:rdf_register_ns(knowrob, 'http://ias.cs.tum.edu/kb/knowrob.owl#', [keep(true)]). 

:- rdf_register_prefix(knowrob, 'http://knowrob.org/kb/knowrob.owl#').
:- rdf_register_prefix(iai-map, 'http://knowrob.org/kb/IAI-kitchen.owl#').

:- rdf_meta 
  getQuaternion(r,-),
  getTranslation(r,-),
  getBoundingBox(r,-).


getFixedKitchenObjects2(ObjectName, Translation, Quaternion, BoundingBox, MeshPath):-
 rdfs_individual_of(Object, knowrob:'SemanticMapPerception'),
 rdf_has(Object, knowrob:'objectActedOn',ObjectName),
 rdf_has(Object, knowrob:'eventOccursAt', Transformation),
 getTranslation(Transformation, Translation),
 getQuaternion(Transformation, Quaternion),
 getBoundingBox(ObjectName, BoundingBox).

getQuaternion(TransformationHandle, Quaternion):-
  rdf(TransformationHandle, knowrob:'quaternion', QuaternionRaw),
  strip_literal_type(QuaternionRaw, QuaternionSpaceSeperated),
  string_to_list(QuaternionSpaceSeperated, Quaternion).
  
getTranslation(TransformationHandle, Translation):-
  rdf(TransformationHandle, knowrob:'translation', TranslationRaw),
  strip_literal_type(TranslationRaw, TranslationSpaceSeperated),
  string_to_list(TranslationSpaceSeperated, Translation).
  
getBoundingBox(BoundingBoxHandle, BoundingBox):-
  rdf(BoundingBoxHandle, rdf:'type', Class),
  owl_direct_subclass_of(Class, Sub),
  owl_restriction(Sub, restriction(knowrob:'boundingBoxSize',_)),
  owl_restriction_object_domain(Sub, BoundingBoxRaw),
  strip_literal_type(BoundingBoxRaw, BoundingBoxSpaceSeperated),
  string_to_list(BoundingBoxSpaceSeperated, BoundingBox).

string_to_list(StringValue, List):-
  atomic_list_concat(StringList,' ', StringValue),
  to_double_list(StringList, [], List).

to_double_list([], TempList, DoubleList):-
  append([], TempList, DoubleList).

to_double_list([H|T], TempList, DoubleList):-
  atom_number(H, DoubleValue),
  append(TempList, [DoubleValue], NewTempList),
  to_double_list(T, NewTempList, DoubleList).
  










getFixedKitchenObjects(Object, M00, M01, M02, M03, M10, M11, M12, M13, M20, M21, M22, M23, M30, M31, M32, M33, Width, Height, Depth) :- 
  rdf_has(A,knowrob:'objectActedOn',Object),
  rdfs_individual_of(A, knowrob:'SemanticMapPerception'),
  rdf_has(Object, knowrob:'heightOfObject', OHeight),
  strip_literal_type(OHeight, Height_Raw),
  atom_number(Height_Raw, Height),
  rdf_has(Object, knowrob:'depthOfObject', ODepth),
  strip_literal_type(ODepth, Depth_Raw),
  atom_number(Depth_Raw, Depth),
  rdf_has(Object, knowrob:'widthOfObject', OWidth),
  strip_literal_type(OWidth, Width_Raw),
  atom_number(Width_Raw, Width),
  rdf_has(A, knowrob:'eventOccursAt',RotationMatrix),
  rdf_has(RotationMatrix, knowrob:'m00', M00_R),
  strip_literal_type(M00_R , M00_s),
  atom_number(M00_s, M00),
  rdf_has(RotationMatrix, knowrob:'m01', M01_R),
  strip_literal_type(M01_R , M01_s),
  atom_number(M01_s, M01),
  rdf_has(RotationMatrix, knowrob:'m02', M02_R),
  strip_literal_type(M02_R , M02_s),
  atom_number(M02_s, M02),
  rdf_has(RotationMatrix, knowrob:'m03', M03_R),
  strip_literal_type(M03_R , M03_s),
  atom_number(M03_s, M03),
  rdf_has(RotationMatrix, knowrob:'m10', M10_R),
  strip_literal_type(M10_R , M10_s),
  atom_number(M10_s, M10),
  rdf_has(RotationMatrix, knowrob:'m11', M11_R),
  strip_literal_type(M11_R , M11_s),
  atom_number(M11_s, M11),
  rdf_has(RotationMatrix, knowrob:'m12', M12_R),
  strip_literal_type(M12_R , M12_s),
  atom_number(M12_s, M12),
  rdf_has(RotationMatrix, knowrob:'m13', M13_R),
  strip_literal_type(M13_R , M13_s),
  atom_number(M13_s, M13),
  rdf_has(RotationMatrix, knowrob:'m20', M20_R),
  strip_literal_type(M20_R , M20_s),
  atom_number(M20_s, M20),
  rdf_has(RotationMatrix, knowrob:'m21', M21_R),
  strip_literal_type(M21_R , M21_s),
  atom_number(M21_s, M21),
  rdf_has(RotationMatrix, knowrob:'m22', M22_R),
  strip_literal_type(M22_R , M22_s),
  atom_number(M22_s, M22),
  rdf_has(RotationMatrix, knowrob:'m23', M23_R),
  strip_literal_type(M23_R , M23_s),
  atom_number(M23_s, M23),
  rdf_has(RotationMatrix, knowrob:'m30', M30_R),
  strip_literal_type(M30_R , M30_s),
  atom_number(M30_s, M30),
  rdf_has(RotationMatrix, knowrob:'m31', M31_R),
  strip_literal_type(M31_R , M31_s),
  atom_number(M31_s, M31),
  rdf_has(RotationMatrix, knowrob:'m32', M32_R),
  strip_literal_type(M32_R , M32_s),
  atom_number(M32_s, M32),
  rdf_has(RotationMatrix, knowrob:'m33', M33_R),
  strip_literal_type(M33_R , M33_s),
  atom_number(M33_s, M33).