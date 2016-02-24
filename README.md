# rosbag_remote_record

A tiny tool to remotely trigger rosbag record

Example:

	python rosbag_remote_record.py -m ros -i /xtion/rgb/image_raw /something/else /another/topic -f testfile


Also look at:

    python rosbag_remote_record.py --help


Remote Listen Scope/Topic:

	/meka/rosbagremote/record


Simply send a Bool value [ true = start recording | false = stop recording] to the above topic/scope
