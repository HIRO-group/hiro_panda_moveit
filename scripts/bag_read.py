import rosbag
bag = rosbag.Bag('joint_states.bag')
for topic, msg, t in bag.read_messages(topics=['joint_states']):
    print(msg)
bag.close()
