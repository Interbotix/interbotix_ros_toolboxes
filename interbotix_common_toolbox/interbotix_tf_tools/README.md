# Interbotix TF Tools

This package contains various tools used manipulate the TF tree by other Interbotix packages.

## tf_rebroadcaster

This tool allows users to rebroadcast specific TFs from one topic to another. For example, say that, for some reason, the user had a TF tree that was published to the topic `/mobile_base/tf` and instead wanted it to be published to `/tf`. This node, along with its configuration file (the template can found at [config/tf_rebroadcaster.yaml](config/tf_rebroadcaster.yaml)) will allow the user to do just that.

### Parameters

The tool takes three parameters:

1. `filepath_config`: (string) absolute filepath to the configuration file.
2. `topic_from`: (string) topic from which the TFs will be retrieved.
3. `topic_to`: (string) topic to which the TF messages will be re-broadcasted.


### Configuration File

```yaml
# The 'frames' YAML node takes mappings between frames in a TF.
# Each mapping represents a TF from the first frame to the second frame.
# For example, a mapping of "odom: base_link" will re-broadcast the
#   odom->base_link transformation to the new topic.
frames:
  odom: base_link
  odom: base_footprint
```

### Launch File

This tool comes with a launch file to make usage easier. An example ros2 launch command would be:

```bash
ros2 launch interbotix_tf_tree tf_rebroadcaster.launch.py topic_from:=/mobile_base/tf topic_to:=/tf
```
