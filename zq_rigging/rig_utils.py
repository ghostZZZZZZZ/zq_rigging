#!/usr/bin/env python
# -*-coding:utf-8 -*-
###########################################
# Time  : 2021.12.08
# Author: zhangqiang 
# Email : 504725439@qq.com 
###########################################

import maya.cmds as cmds
import pymel.core as pm
import maya.api.OpenMaya as om
import json
from utils import *


def auto_joint_orient():
    pass

CTRL_SCALE = 1.0

def set_ctrl_scale(scale):
    global CTRL_SCALE
    CTRL_SCALE = scale

class CtrlCreate(object):

    def __init__(self,name,matrix=None,ctrl_type="circle",scale=1.0):
        global CTRL_SCALE
        self.name = name
        self.ctrl = None
        self.offset = None
        self.group = None
        self.dummy = None
        self.matrix = matrix
        self.ctrl_type= ctrl_type
        CTRL_SCALE = scale

        self.ctrl_create()
    
    # @classmethod
    # def create(cls,name,ctrl_type="circle"):
    #     return cls(name,ctrl_type).ctrl_create()

    def ctrl_create(self):

        self.ctrl = self.create_ctrl_curve()
        self.ctrl = pm.PyNode(self.ctrl.partialPathName())
        self.ctrl.rename(self.name)
        self.ctrl.rotateOrder.setKeyable(1)
        self.group = pm.group(self.ctrl,name=self.name + "_group")
        self.offset = pm.group(self.group,name=self.name + "_offset")
        self.dummy = pm.group(em=1,p=self.ctrl,name=self.name + "_dummy")
        if self.matrix:
            self.offset.setMatrix(self.matrix,ws=1)


    def create_ctrl_curve(self):

        shape_dir = get_control_shapes_dir()
        shape_file = os.path.join(shape_dir,"{}.shape".format(self.ctrl_type))

        with open(shape_file,"r") as f:
            shapeData = json.load(f)
        
        parentInverseMatrix = om.MMatrix([  0,-1,0,1,
                                            1,0,0,1,
                                            0,0,1,1,
                                            0,0,0,1])
        parentInverseMatrix *= CTRL_SCALE
        parent = om.MObject.kNullObj

        newCurve = om.MFnNurbsCurve()
        newShapes = []
        for shapeName, curveData in iter(shapeData.items()):
            cvs = om.MPointArray(curveData["cvs"])  # om allows a list of lists which converts to om.Point per element
            knots = curveData["knots"]
            degree = curveData["degree"]
            form = curveData["form"]
            enabled = curveData["overrideEnabled"]
            # if space == om.MSpace.kWorld and parent != om.MObject.kNullObj:
            for i in range(len(cvs)):
                cvs[i] *= parentInverseMatrix
            shape = newCurve.create(cvs, knots, degree, form, False, False, parent)
            newShapes.append(shape)
            if parent == om.MObject.kNullObj and shape.apiType() == om.MFn.kTransform:
                parent = shape
            if enabled:
                newCurve.findPlug("overrideEnabled", False).setInt(int(curveData["overrideEnabled"]))
                colours = curveData["overrideColorRGB"]
        return om.MFnTransform(parent)

    
    