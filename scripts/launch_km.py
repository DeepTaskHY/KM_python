#!/usr/bin/env python
# -*- coding: utf-8 -*-

from rosnode import KMNode

if __name__ == "__main__":
    km_node = KMNode(node_name='km_node')
    km_node.spin()
