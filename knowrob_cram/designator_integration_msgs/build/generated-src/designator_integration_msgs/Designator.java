package designator_integration_msgs;

public interface Designator extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "designator_integration_msgs/Designator";
  static final java.lang.String _DEFINITION = "int32 TYPE_OBJECT=0\nint32 TYPE_ACTION=1\nint32 TYPE_LOCATION=2\nint32 TYPE_HUMAN=3\n\nint32 type\n\nKeyValuePair[] description\n";
  static final int TYPE_OBJECT = 0;
  static final int TYPE_ACTION = 1;
  static final int TYPE_LOCATION = 2;
  static final int TYPE_HUMAN = 3;
  int getType();
  void setType(int value);
  java.util.List<designator_integration_msgs.KeyValuePair> getDescription();
  void setDescription(java.util.List<designator_integration_msgs.KeyValuePair> value);
}
