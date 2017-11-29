package designator_integration_msgs;

public interface DesignatorResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "designator_integration_msgs/DesignatorResponse";
  static final java.lang.String _DEFINITION = "Designator[] designators\n";
  java.util.List<designator_integration_msgs.Designator> getDesignators();
  void setDesignators(java.util.List<designator_integration_msgs.Designator> value);
}
