#!/usr/bin/env python
# -*-coding:utf-8 -*-
###########################################
# Time  : 2021.12.08
# Author: zhangqiang 
# Email : 504725439@qq.com 
###########################################


import pymel.core as pm
import maya.cmds as cmds
import os




def get_base_dir():
    base_dir = ""
    try:
        base_dir = os.path.dirname(__file__)
    except:
        pass
    return base_dir
def get_control_shapes_dir():
    return os.path.join(get_base_dir(),"control_shapes")


def quickCreateNode(**kwargs):
    name=None
    if kwargs.has_key("name"):
        name=kwargs.pop("name")
    nodeType = kwargs.pop("type")
    if name:
        result_node = pm.createNode(nodeType,name=name)
    else:
        result_node = pm.createNode(nodeType)
    for i,v in kwargs.items():
        attr = getattr(result_node,i)
        if type(v)==pm.general.Attribute:
            v.connect(attr)
        elif type(v) == str:
            pm.connectAttr(v,attr,f=1)
        else:
            attr.set(v)
    return result_node