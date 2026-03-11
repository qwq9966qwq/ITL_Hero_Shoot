#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import sys
import xml.dom.minidom
from .sdf_util import parse_model_uri

def get_child_node_by_tag(node, name):
    for child in node.childNodes:
        if child.nodeType == xml.dom.Node.ELEMENT_NODE and child.tagName == name:
            return child
    return None

def create_child_node(doc, parent_node, tag_name):
    new_node = doc.createElement(tag_name)
    parent_node.appendChild(new_node)
    return new_node

def convert_pose(in_node, out_doc, out_parent_node):
    new_node = create_child_node(out_doc, out_parent_node, 'origin')
    pose = in_node.firstChild.data.split(' ')
    if len(pose) == 6:
        new_node.setAttribute('rpy', pose[3] + ' ' + pose[4] + ' ' + pose[5])
        new_node.setAttribute('xyz', pose[0] + ' ' + pose[1] + ' ' + pose[2])
    else:
        print("convert_pose error", file=sys.stderr)

def convert_geometry(in_node, out_doc, out_parent_node):
    new_node = create_child_node(out_doc, out_parent_node, 'geometry')
    for child in in_node.childNodes:
        if child.nodeType == xml.dom.Node.ELEMENT_NODE:
            if child.tagName == 'mesh':
                uri_node = get_child_node_by_tag(child, 'uri')
                uri = 'file://'+parse_model_uri(uri_node.firstChild.data)
                new_mesh_node = create_child_node(out_doc, new_node, 'mesh')
                new_mesh_node.setAttribute('filename', uri)
                return True
            elif child.tagName == 'box':
                size_node = get_child_node_by_tag(child, 'size')
                new_box_node = create_child_node(out_doc, new_node, 'box')
                new_box_node.setAttribute('size', size_node.firstChild.data)
                return True
            elif child.tagName == 'cylinder':
                radius_node = get_child_node_by_tag(child, 'radius')
                length_node = get_child_node_by_tag(child, 'length')
                new_cylinder_node = create_child_node(out_doc, new_node, 'cylinder')
                new_cylinder_node.setAttribute('radius', radius_node.firstChild.data)
                new_cylinder_node.setAttribute('length', length_node.firstChild.data)
                return True
            elif child.tagName == 'sphere':
                radius_node = get_child_node_by_tag(child, 'radius')
                new_sphere_node = create_child_node(out_doc, new_node, 'sphere')
                new_sphere_node.setAttribute('radius', size_node.firstChild.data)
                return True
    return False

def convert_inertial(in_node, out_doc, out_parent_node):
    new_node = create_child_node(out_doc, out_parent_node, 'inertial')
    for child in in_node.childNodes:
        if child.nodeType == xml.dom.Node.ELEMENT_NODE:
            if child.tagName == 'pose':
                convert_pose(child, out_doc, new_node)
            if child.tagName == 'mass':
                new_mass_node = create_child_node(out_doc, new_node, 'mass')
                new_mass_node.setAttribute('value', child.firstChild.data)
            elif child.tagName == 'inertia':
                new_inertia_node = create_child_node(out_doc, new_node, 'inertia')
                for c in child.childNodes:
                    if c.nodeType == xml.dom.Node.ELEMENT_NODE:
                        new_inertia_node.setAttribute(c.tagName, c.firstChild.data)

# only support diffuse
def convert_material(in_node, out_doc, out_parent_node):
    diffuse_node = get_child_node_by_tag(in_node, 'diffuse')
    if diffuse_node is not None:
        new_node = create_child_node(out_doc, out_parent_node, 'material')
        new_node.setAttribute('name', '')
        new_color_node = create_child_node(out_doc, new_node, 'color')
        rgba = diffuse_node.firstChild.data
        if len(rgba.split(' ')) == 3:
            rgba = rgba + ' 1.0'
        new_color_node.setAttribute('rgba', rgba)

def convert_visual(in_node, out_doc, out_parent_node):
    new_node = out_doc.createElement('visual')
    for child in in_node.childNodes:
        if child.nodeType == xml.dom.Node.ELEMENT_NODE:
            if child.tagName == 'pose':
                convert_pose(child, out_doc, new_node)
            elif child.tagName == 'geometry':
                if not convert_geometry(child, out_doc, new_node):
                    return
            elif child.tagName == 'material':
                convert_material(child, out_doc, new_node)
    out_parent_node.appendChild(new_node)

def convert_collision(in_node, out_doc, out_parent_node):
    new_node = out_doc.createElement('collision')
    for child in in_node.childNodes:
        if child.nodeType == xml.dom.Node.ELEMENT_NODE:
            if child.tagName == 'pose':
                convert_pose(child, out_doc, new_node)
            elif child.tagName == 'geometry':
                if not convert_geometry(child, out_doc, new_node):
                    return
    out_parent_node.appendChild(new_node)

def convert_link(in_node, out_doc, out_parent_node):
    new_node = create_child_node(out_doc, out_parent_node, 'link')
    new_node.setAttribute('name', in_node.getAttribute("name"))
    for child in in_node.childNodes:
        if child.nodeType == xml.dom.Node.ELEMENT_NODE:
            if child.tagName == 'pose':
                convert_pose(child, out_doc, new_node)
            elif child.tagName == 'inertial':
                convert_inertial(child, out_doc, new_node)
            elif child.tagName == 'collision':
                convert_collision(child, out_doc, new_node)
            elif child.tagName == 'visual':
                convert_visual(child, out_doc, new_node)

def convert_joint_axis(in_node, out_doc, out_parent_node):
    for child in in_node.childNodes:
        if child.nodeType == xml.dom.Node.ELEMENT_NODE:
            if child.tagName == 'xyz':
                new_node = create_child_node(out_doc, out_parent_node, 'axis')
                new_node.setAttribute('xyz', child.firstChild.data)
            elif child.tagName == 'limit':
                new_node = create_child_node(out_doc, out_parent_node, 'limit')
                for c in child.childNodes:
                    if c.nodeType == xml.dom.Node.ELEMENT_NODE and c.tagName in ['effort', 'lower', 'upper', 'velocity']:
                        new_node.setAttribute(c.tagName, c.firstChild.data)
            elif child.tagName == 'dynamics':
                new_node = create_child_node(out_doc, out_parent_node, 'dynamics')
                for c in child.childNodes:
                    if c.nodeType == xml.dom.Node.ELEMENT_NODE and c.tagName in ['damping', 'friction']:
                        new_node.setAttribute(c.tagName, c.firstChild.data)

def convert_joint(in_node, out_doc, out_parent_node):
    if in_node.getAttribute("type") not in ['fixed', 'continuous', 'prismatic', 'revolute']:
        print("convert_joint error, ignore", file=sys.stderr)
        return
    new_node = create_child_node(out_doc, out_parent_node, 'joint')
    new_node.setAttribute('name', in_node.getAttribute("name"))
    new_node.setAttribute('type', in_node.getAttribute("type"))
    for child in in_node.childNodes:
        if child.nodeType == xml.dom.Node.ELEMENT_NODE:
            if child.tagName == 'pose':
                convert_pose(child, out_doc, new_node)
            elif child.tagName == 'parent':
                new_parent_node = create_child_node(out_doc, new_node, 'parent')
                new_parent_node.setAttribute('link', child.firstChild.data)
            elif child.tagName == 'child':
                new_child_node = create_child_node(out_doc, new_node, 'child')
                new_child_node.setAttribute('link', child.firstChild.data)
            elif child.tagName == 'axis':
                convert_joint_axis(child, out_doc, new_node)

def sdf_to_urdf(in_doc):
    in_root = get_child_node_by_tag(in_doc.documentElement, 'model')
    out_doc = xml.dom.minidom.Document()
    out_root = out_doc.createElement('robot')
    out_doc.appendChild(out_root)
    out_root.setAttribute('name', in_root.getAttribute("name"))
    for node in in_root.childNodes:
        if node.nodeType == xml.dom.Node.ELEMENT_NODE:
            if node.tagName == 'link':
                convert_link(node, out_doc, out_root) 
            elif node.tagName == 'joint':
                convert_joint(node, out_doc, out_root)
    return out_doc

def sdf2urdf_main():
    if(len(sys.argv) < 2):
        print("Usage: sdf2urdf <sdf file>")
        return -1
    inputfile = sys.argv[1]
    # check
    if os.path.splitext(inputfile)[1] != '.sdf':
        print("Error: the name of inputfile must be xxx.sdf")
        return -2  
    # process
    in_doc = xml.dom.minidom.parse(inputfile)
    out_doc = sdf_to_urdf(in_doc)
    try:
        print(out_doc.toprettyxml())
    except Exception as e:
        print("Error:",e)
    return 0

if __name__ == '__main__':
    sdf2urdf_main()
