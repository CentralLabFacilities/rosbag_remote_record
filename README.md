# rosbag_remote_record

A tiny tool to remotely trigger rosbag record through ROS or RSB topic trigger

## Example:

### running the app
```bash
python rosbag_remote_record.py -m ros -i /xtion/rgb/image_raw /something/else /another/topic -f testfile
```

### triggering the recording start/stop with ROS

```bash
rostopic pub /meka/rosbagremote/record std_msgs/Bool True
rostopic pub /meka/rosbagremote/record std_msgs/Bool False
```

### triggering the recording start/stop with RSB

```bash
rsb-toolscl0.15 send 'true' 'socket:/meka/rosbagremote/record'
rsb-toolscl0.15 send 'false' 'socket:/meka/rosbagremote/record'
```

Also look at:


```bash
python rosbag_remote_record.py --help
```


## Setting up

### Default Remote Listen Scope/Topic:

```bash
/meka/rosbagremote/record
```

for user specified Remote Listen Scope/Topic use -t option

### Default mode (ROS and RSB)

  * Simply send a Bool value [ true = start recording | false = stop recording] to the above topic/scope
  
### Advanced mode (only in ROS now)

  * Filename can be specified for each start/stop operation
  * Use a String value containing [ filename:start | filename:stop ] to the extended trigger topic/scope, to be start/stop recording file called filename
  * the extended trigger topic/scope is the standard topic with ```/named``` at the end

```bash
    rostopic pub /trigger/named std_msgs/String 'myfile:start'
    rostopic pub /trigger/named std_msgs/String 'myfile:stop'
```
