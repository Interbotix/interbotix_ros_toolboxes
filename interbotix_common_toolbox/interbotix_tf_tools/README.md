# Interbotix TF Tools

This package contains various tools used manipulate the TF tree.

## tf_rebroadcaster

This tool, built as a composable node, allows users to rebroadcast specific TFs from one topic to another and optionally add a prefix. For example, say that, for some reason, the user had a TF tree that was published to the topic `/mobile_base/tf` and instead wanted it to be published to `/tf`. This node, along with its configuration file (the template can found at [config/tf_rebroadcaster.yaml](config/tf_rebroadcaster.yaml)) will allow the user to do just that.

### Parameters

The tool takes three parameters:

1. `filepath_config`: (string) absolute filepath to the configuration file.
2. `topic_from`: (string) topic from which the TFs will be retrieved.
3. `topic_to`: (string) topic to which the TF messages will be re-broadcasted.

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
ros2 launch interbotix_tf_tools tf_rebroadcaster.launch.py topic_from:="/tf/mobile_base" topic_to:="/tf"
```

You may include the tf_rebroadcaster in a `ComposableNodeContainer` like so:

```python
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

tf_rebroadcaster_container = ComposableNodeContainer(
    name='tf_rebroadcaster_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container',
    composable_node_descriptions=[
        ComposableNode(
            package='interbotix_tf_tools',
            plugin='interbotix_tf_tools::TFRebroadcaster',
            name='tf_rebroadcaster',
            parameters=[
                {
                    'filepath_config': config_launch_arg,
                    'topic_from': topic_from_launch_arg,
                    'topic_to': topic_to_launch_arg,
                },
            ],
        ),
        ComposableNode(
            ...
        ),
    ]
)
```
