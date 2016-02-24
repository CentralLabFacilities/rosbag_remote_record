# rosbag_remote_record

A tiny tool to remotely trigger rosbag record

Example

	python rosbag_remote_record.py -m rsb -i /xtion/rgb/image_raw /something/else /another/topic -f testfile


Remote Listen Scope/Topic

	/meka/rosbagremote/record

Simply send a Bool value [true=start recording | false=stop recording] to the above topic/scope
