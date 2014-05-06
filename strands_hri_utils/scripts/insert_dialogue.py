#!/usr/bin/env python

import sys
import std_msgs.msg
from ros_datacentre.message_store import MessageStoreProxy

def loadDialogue(inputfile, dataset_name) :
    print "openning %s" %inputfile
    with open(inputfile) as f:
            content = f.readlines()
    print "Done"
    return content


if __name__ == '__main__':
    if len(sys.argv) < 3 :
        print "usage: insert_dialogue input_file.txt dataset_name"
	sys.exit(2)

    filename = str(sys.argv[1])
    dataset_name = str(sys.argv[2])
    msg_store = MessageStoreProxy(collection="hri_behaviours")
    sentences = loadDialogue(filename, dataset_name)
    for idx, element in enumerate(sentences):
        sentence = std_msgs.msg.String()
        sentence.data = str(element)
        meta = {}
        meta["hri_dialogue"] = dataset_name
        meta["index"] = idx
        msg_store.insert(sentence,meta)
