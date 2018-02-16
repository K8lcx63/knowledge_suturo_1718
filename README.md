## SUTURO 1718 Knowledgepackage

### Installation

- Java8 installieren: [klick mich](https://wiki.ubuntuusers.de/Java/Installation/Oracle_Java/Java_8/) **Wichtig JDK**
- Repo clonen und bauen
- Die folgenden Schritte ausführen: [klick mich](http://knowrob.org/installation/workspace)
- rosdep install --from-paths \<pfad zum catkin_ws> --ignore-src --rosdistro=indigo -r -y **(z.b ~/catkin_ws)**
- sudo apt-get update 

### Launchen

Um Alle Knoten des Knowledgepackages zu starten wird folgender Befehl verwendet:

 > roslaunch knowledge knowledge_main.launch


### Services

#### knowledge_msgs/GetFixedKitchenObjects

Topic-Name: /kitchen_model_service/get_fixed_kitchen_objects  
Eingabe:  
Rückgabe: string frame_id  
&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; string[] names  
&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; geometry_msgs/Pose[] poses  
&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; geometry_msgs/Vector3[] bounding_boxes(width, height, depth) 

Dieser Service exportiert alle festen Objecte im Lab aus der 'room.owl' in das in der Rückgabe angegebene Format. Das Ziel ist es diese Daten an Motion weiterzureichen und daraus eine Planning-Scene zu erstellen, damit der Roboter seine Umgebung kennt.

#### knowledge_msgs/Classify

Topic-Name: /svm_classifier/classify  
Eingabe:  float64[] features 
Rückgabe: string label 

Dieser Service nimmt ein 1d-Feature-Array als Eingabe. In dem Array steht zu erst ein Colorhistogram gefolgt von einem Normalshistogram. Mithilfe einer SVM wird dann das label vorhergesagt.

#### knowledge_msgs/StoragePlace

Topic-Name: /kitchen_model_service/get_fixed_kitchen_objects  
Eingabe: string object_label  
Rückgabe: geometry_msgs/PointStamped storage_place_position  
&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; float64 storage_place_width  
&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; gfloat64 storage_place_height

Dieser Service entscheidet, welches Objekt an welchen Lagerplatz gehört. Es gibt folgende Objekte und Objektklassen:

| Objektklasse| Objekte          
| ------------|:-------------------------------------------------------| 
| Saucen      | HelaCurryKetchup, TomatoSauceOroDiParma                 | 
| Snacks      | PringlesPaprika, PringlesSalt                           |  
| Frühstück   | JaMilch, KoellnMuesliKnusperHonigNuss, KellogsToppasMini|
| Geschirr    | CupEcoOrange, EdekaRedBowl, SiggBottle                  |

#### knowledge_msgs/EmptyGripper

Topic-Name: /beliefstate/gripper_empty  
Eingabe:  
Rückgabe: bool left_gripper  
&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; bool right_gripper

Dieser Service gibt an welche der beiden Gripper frei ist. Sollte sich ein Object im linken Gripper befinden und im rechten nicht, dann wäre left_gripper == true und right_gripper == false.

#### knowledge_msgs/ObjectsToPick

Topic-Name: /beliefstate/objects_to_pick  
Eingabe:  
Rückgabe: string object_label_1  
&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; string object_label_2

Dieser Service entscheidet, welche zwei Objekte als nächstes in die Lagerplätze einsortiert werden. Dabei  wird versucht, dass zwei Objekte gewählt werden, welche an den gleichen Lagerplatz gehören. Ist dies nicht möglich, dann werden zwei beliebige Objekte gewählt. Sollte es nur noch ein Objekt geben, dass einsortiert werden muss, dann ist object_label_2 der leere string und sollte es keine Objekte mehr geben, dann ist object_label_1 und object_label_2 mit dem leeren string belegt. 

### Topics

#### knowledge_msgs/PerceiveObject

Topic-Name: /beliefstate/perceive_action  
Message: string object_label  
&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;geometry_msgs/PoseStamped object_pose 

Auf diesem Topic muss immer dann eine Nachticht gepublisht werden, wenn ein Objekt von der Perception wahrgenommen wird.

#### knowledge_msgs/GraspObject

Topic-Name: /beliefstate/grasp_action  
Message: string object_label  
&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; knowledge_msgs/Gripper gripper 

Auf diesem Topic muss immer dann eine Nachticht gepublisht werden, wenn ein Objekt gegriffen wird. Genauer gesagt, wenn der Gripper geschlossen wurde.

#### knowledge_msgs/DropObject

Topic-Name: /beliefstate/drop_action  
Message: knowledge_msgs/Gripper gripper

Auf diesem Topic muss immer dann eine Nachticht gepublisht werden, wenn ein Objekt losgelassen wird. Genauer gesagt, bevor der Gripper geöffnet wird.