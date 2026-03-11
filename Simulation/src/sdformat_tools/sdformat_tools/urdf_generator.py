#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import sys
import re
import xml.dom.minidom
from .sdf2urdf import sdf_to_urdf

class UrdfGenerator():
    def __init__(self):
        self.out_doc = None

    def __remove_node(self, tag_name, name):
        if self.out_doc is None:
            return
        root = self.out_doc.documentElement
        for node in root.childNodes:
            if node.nodeType == xml.dom.Node.ELEMENT_NODE \
                and node.tagName == tag_name \
                and node.getAttribute("name") == name :
                root.removeChild(node)

    def __merge_urdf_doc(self, doc):
        if self.out_doc is None:
            return
        root = doc.documentElement
        for node in root.childNodes:
            if node.nodeType == xml.dom.Node.ELEMENT_NODE:
                self.out_doc.documentElement.appendChild(node)

    def parse_from_urdf_string(self, xml_str):
        self.out_doc = xml.dom.minidom.parseString(xml_str)
        if self.out_doc.documentElement.tagName != 'robot':
            raise Exception("[line %d]"%(sys._getframe().f_lineno)+" it's not a urdf, please check it.")

    def parse_from_urdf_file(self, filepath):
        with open(filepath, "r") as f:
            self.parse_from_urdf_string(f.read())

    def parse_from_sdf_string(self, xml_str):
        in_doc = xml.dom.minidom.parseString(xml_str)
        if in_doc.documentElement.tagName != 'sdf':
            raise Exception("[line %d]"%(sys._getframe().f_lineno)+" it's not a sdf, please check it.")
        self.out_doc = sdf_to_urdf(in_doc)

    def parse_from_sdf_file(self, filepath):
        with open(filepath, "r") as f:
            self.parse_from_sdf_string(f.read())

    def merge_urdf_file(self, filepath):
        doc = xml.dom.minidom.parse(filepath)
        self.__merge_urdf_doc(doc)

    def merge_urdf_string(self, xml_str):
        doc = xml.dom.minidom.parseString(xml_str)
        self.__merge_urdf_doc(doc)

    def remove_link(self, link_name):
        self.__remove_node('link', link_name)

    def remove_joint(self, joint_name):
        self.__remove_node('joint', joint_name)

    def to_string(self):
        if self.out_doc is None:
            return ""
        return self.out_doc.toprettyxml()

    def to_file(self,filepath):
        xml = self.to_string()
        # write to file
        with open(filepath, 'w', encoding='UTF-8') as f:
            f.write(xml)
