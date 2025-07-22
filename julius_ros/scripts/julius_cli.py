#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


from argparse import ArgumentParser
import rospy
from julius_ros.cli import register_isolated
from julius_ros.cli import register_grammar


if __name__ == '__main__':
    rospy.init_node("julius_cli")

    p = ArgumentParser()
    p.add_argument("-i", "--isolated", help="isolated word recognition",
                   action='store_true')
    p.add_argument("-n", "--name", help="name of grammar / vocabulary",
                   type=str)
    p.add_argument("path_or_words", nargs='+', type=str)

    print(rospy.myargv()[1:])

    args = p.parse_args(rospy.myargv()[1:])

    print(args)

    if args.isolated:
        if len(args.path_or_words) < 1:
            p.error("Need at least one word to register")
        words = args.path_or_words
        register_isolated(args.name, words)
    else:
        if len(args.path_or_words) != 1:
            p.error("Only one path supported")
        path = args.path_or_words[0]
        register_grammar(args.name, path)
