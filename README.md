## SUTURO 1718 Knowledgepackage

### Installation

- Java8 installieren: [klick mich](https://wiki.ubuntuusers.de/Java/Installation/Oracle_Java/Java_8/) **Wichtig JDK**
- Repo clonen und bauen

### Services

Folgende Services gibt es:

#### object_detection/PokeObject

Eingabe: ObjectDetection<br/><br/>
Rückgabe: gemetry_msgs/PointStamped

Dieser Service berechnert anhand der Eingabe eine optimale Anstoßposition, so dass das Object zu Fall gebracht wird. Zu beachten ist, dass die Position welche in der Rückgabe gehalten wird absolut ist und in dem Frame sich befindet, welches in header.frame_id steht.