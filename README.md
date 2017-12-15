## SUTURO 1718 Knowledgepackage

### Installation

- Java8 installieren: [klick mich](https://wiki.ubuntuusers.de/Java/Installation/Oracle_Java/Java_8/) **Wichtig JDK**
- Repo clonen und bauen
- Die folgenden Schritte ausführen: [klick mich](http://knowrob.org/installation/workspace)
- rosdep install --from-paths <pfad zum catkin_ws> --ignore-src --rosdistro=indigo(z.b ~/catkin_ws)

### Launchen

Um Alle Knoten des Knowledgepackages zu starten wird folgender Befehl verwendet:

 > roslaunch knowledge knowledge_main.launch


### Services

Folgende Services gibt es:

#### object_detection/PokeObject

Topic-Name: /poke_position_service/calculate_poke_position  
Eingabe: ObjectDetection    
Rückgabe: geometry_msgs/PointStamped poke_position

Dieser Service berechnert anhand der Eingabe eine optimale Anstoßposition, so dass das Object zu Fall gebracht wird. Zu beachten ist, dass die Position welche in der Rückgabe gehalten wird absolut ist und in dem Frame sich befindet, welches in header.frame_id steht.

#### knowledge_msgs/GetFixedKitchenObjects

Topic-Name: /kitchen_model_service/get_fixed_kitchen_objects  
Eingabe:  
Rückgabe: string frame_id  
&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; string[] names  
&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; geometry_msgs/Pose[] poses  
&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; geometry_msgs/Vector3[] bounding_boxes(width, height, depth) 


Dieser Service exportiert alle festen Objecte im Lab aus der 'room.owl' in das in der Rückgabe angegebene Format. Das Ziel ist es diese Daten an Motion weiterzureichen und daraus eine Planning-Scene zu erstellen, damit der Roboter seine Umgebung kennt.