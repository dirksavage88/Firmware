#!/usr/bin/env python3

"""
Generate docs from .msg files
"""

import os
import argparse
import sys


def get_msgs_list(msgdir):
    """
    Makes a list of relative paths of .msg files in the given directory
    and its subdirectories.

    Parameters:
    msgdir (str): The directory to search for .msg files.

    Returns:
    list: A list of relative paths to .msg files.
    """
    msgs = []
    for root, _, files in os.walk(msgdir):
        for fn in files:
            if fn.endswith(".msg"):
                relative_path = os.path.relpath(os.path.join(root, fn), msgdir)
                msgs.append(relative_path)
    return msgs


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate docs from .msg files')
    parser.add_argument('-d', dest='dir', help='output directory', required=True)
    args = parser.parse_args()

    output_dir = args.dir
    if not os.path.isdir(output_dir):
        os.mkdir(output_dir)

    msg_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),"../../msg")
    msg_files = get_msgs_list(msg_path)
    msg_files.sort()

    versioned_msgs_list = ''
    unversioned_msgs_list = ''

    for msg_file in msg_files:
        msg_name = os.path.splitext(os.path.basename(msg_file))[0]
        output_file = os.path.join(output_dir, msg_name+'.md')
        msg_filename = os.path.join(msg_path, msg_file)
        print("{:} -> {:}".format(msg_filename, output_file))

        #Format msg url
        msg_url="[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/%s)" % msg_file

        msg_description = ""
        summary_description = ""

        #Get msg description (first non-empty comment line from top of msg)
        with open(msg_filename, 'r') as lineparser:
            line = lineparser.readline()
            while line.startswith('#') or (line.strip() == ''):
                print('DEBUG: line: %s' % line)
                line=line[1:].strip()+'\n'
                stripped_line=line.strip()
                if msg_description and not summary_description and stripped_line=='':
                    summary_description = msg_description.strip()

                msg_description+=line
                line = lineparser.readline()
            msg_description=msg_description.strip()
            if not summary_description and msg_description:
                summary_description = msg_description
            print('msg_description: Z%sZ' % msg_description)
            print('summary_description: Z%sZ' % summary_description)
            summary_description
        msg_contents = ""
        #Get msg contents (read the file)
        with open(msg_filename, 'r') as source_file:
            msg_contents = source_file.read()

        #Format markdown using msg name, comment, url, contents.
        markdown_output="""# %s (UORB message)

%s

%s

```c
%s
```
""" % (msg_name, msg_description, msg_url, msg_contents)

        with open(output_file, 'w') as content_file:
            content_file.write(markdown_output)

        # Categorize as versioned or unversioned
        if "versioned" in msg_file:
            versioned_msgs_list += '- [%s](%s.md)' % (msg_name, msg_name)
            if summary_description:
                versioned_msgs_list += " — %s" % summary_description
            versioned_msgs_list += "\n"
        else:
            unversioned_msgs_list += '- [%s](%s.md)' % (msg_name, msg_name)
            if summary_description:
                unversioned_msgs_list += " — %s" % summary_description
            unversioned_msgs_list += "\n"

    # Write out the index.md file
    index_text="""# uORB Message Reference

::: info
This list is [auto-generated](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/msg/generate_msg_docs.py) from the source code.
:::

This topic lists the UORB messages available in PX4 (some of which may be may be shared by the [PX4-ROS 2 Bridge](../ros/ros2_comm.md)).

[Versioned messages](../middleware/uorb.md#message-versioning) track changes to their definitions, with each modification resulting in a version increment.
These messages are most likely shared through the PX4-ROS 2 Bridge.

Graphs showing how these are used [can be found here](../middleware/uorb_graph.md).

## Versioned Messages

%s

## Unversioned Messages

%s
    """ % (versioned_msgs_list, unversioned_msgs_list)
    index_file = os.path.join(output_dir, 'index.md')
    with open(index_file, 'w') as content_file:
            content_file.write(index_text)
