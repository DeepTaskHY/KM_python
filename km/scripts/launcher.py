#!/usr/bin/python3
# -*- coding: utf-8 -*-

from rosnode import KMNode


if __name__ == '__main__':
    km_node = KMNode(node_name='KM_node')
    km_node.spin()
