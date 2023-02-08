# Interbotix TF Tools

This package contains various tools used manipulate the TF tree.

## tf_rebroadcaster

This tool, built as a composable node, allows users to rebroadcast specific TFs from one topic to another and optionally add a prefix. For example, say that, for some reason, the user had a TF tree that was published to the topic `/mobile_base/tf` and instead wanted it to be published to `/tf`. This node, along with its configuration file (the template can found at [config/tf_rebroadcaster.yaml](config/tf_rebroadcaster.yaml)) will allow the user to do just that.

### Parameters

The tool takes four parameters:

1. `filepath_config`: (string) absolute filepath to the configuration file.
2. `topic_from`: (string) topic from which the TFs will be retrieved.
3. `topic_to`: (string) topic to which the TF messages will be re-broadcasted.
4. `use_incoming_time`: (bool) whether or not to use the timestamp of the incoming TF message.

### Configuration File Format

```yaml
# The 'frames' node contains all frames to be rebroadcasted.
# Each frame is formatted as so:
# frames:
#   PARENT_FRAME_ID:
#     child_frame_id: CHILD_FRAME_ID
#     prefix: PREFIX
# This configuration would take the transform from frame 'PARENT_FRAME_ID' to
#   frame 'CHILD_FRAME_ID', add the prefix 'PREFIX', and rebroadcast it to the
#   specified topic.
# If prefix is left as an empty string (''), no prefix will be added.
frames:
  odom:
    child_frame_id: base_link
    prefix: ''
  odom:
    child_frame_id: base_footprint
    prefix: ''
```

### Demo Launch File

This tool comes with a launch file to demonstrate usage. An example ros2 launch command would be:

```bash
roslaunch interbotix_tf_tools tf_rebroadcaster.launch topic_from:="/tf/mobile_base" topic_to:="/tf"
```

An example launch file is provided for a usage example.

```xml
<launch>

  <arg name="topic_to"                    default=""/>
  <arg name="topic_from"                  default=""/>
  <arg name="tf_rebroadcaster_config"     default="$(find interbotix_tf_tools)/config/tf_rebroadcaster.yaml"/>
  <arg name="use_incoming_time"           default="true"/>

  <node
    name="tf_rebroadcaster"
    pkg="interbotix_tf_tools"
    type="tf_rebroadcaster"
    output="screen">
    <param name="topic_to"                value="$(arg topic_to)"/>
    <param name="topic_from"              value="$(arg topic_from)"/>
    <param name="filepath_config"         value="$(arg tf_rebroadcaster_config)"/>
    <param name="use_incoming_time"       value="$(arg use_incoming_time)"/>
  </node>

</launch>
```
