#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import sys
import re
import xml.dom.minidom
import xmacro.xml_format
from xmacro.xmacro import XMLMacro
from .sdf_util import parse_model_uri

class XMLMacro4sdf(XMLMacro):
    def __init__(self):
        super().__init__()
        self.tool_name = "xmacro4sdf"
        self.parse_uri_fn = parse_model_uri
        filepath = os.path.join(os.path.dirname(os.path.abspath(__file__)) ,'common.sdf.xmacro')
        self.common_xmacro_paths.append(filepath)

def xmacro4sdf_main():
    if(len(sys.argv) < 2):
        print("Usage: xmacro4sdf <inputfile> (the name of inputfile must be xxx.xmacro)")
        return -1
    inputfile = sys.argv[1]
    # check
    if os.path.splitext(inputfile)[1] != '.xmacro':
        print("Error: the name of inputfile must be xxx.xmacro")
        return -2  
    # process 
    xmacro=XMLMacro4sdf()
    xmacro.set_xml_file(inputfile)
    try:
        xmacro.generate()
        print(xmacro.to_string())
    except Exception as e:
        print("Error:",e)
    return 0

if __name__ == '__main__':
    xmacro4sdf_main()