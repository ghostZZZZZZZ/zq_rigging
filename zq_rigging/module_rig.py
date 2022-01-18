#!/usr/bin/env python
# -*-coding:utf-8 -*-
###########################################
# Time  : 2021.12.08
# Author: zhangqiang 
# Email : 504725439@qq.com 
###########################################


import pymel.core as pm
import maya.cmds as cmds
import json
from rig_utils import *

class three_jnt_Ik():

    def __init__(self,jnts,name):
        self.jnts = jnts
        self.name =name
        self.handle_ctrl = None
        self.poleVector_ctrl = None

    @property
    def ctrl_offsets(self):

        return [i.offset for i in [self.poleVector_ctrl,self.handle_ctrl]]

    def get_pole_matrix(self):
        jnt_poss = [i.getTranslation(space="world") for i in self.jnts]
        f_dist = jnt_poss[0].distanceTo(jnt_poss[1])
        e_dist = jnt_poss[1].distanceTo(jnt_poss[2])


        back_point = jnt_poss[0] * (e_dist / (f_dist + e_dist)) + jnt_poss[2] * (f_dist / (f_dist + e_dist))
        vec = jnt_poss[1] - back_point
        vec.normalize()
        back_point += vec * f_dist
        matrix = pm.dt.Matrix()
        matrix[-1] = [back_point[0],back_point[1],back_point[2],1]
        return matrix

    def create_ctrls(self):

        hand_matrix = pm.dt.Matrix()
        hand_matrix[-1] = self.jnts[-1].getMatrix(ws=1)[-1]
        self.handle_ctrl = CtrlCreate(self.name + "_IK_Handle_Ctrl",matrix=hand_matrix,ctrl_type="cube")
        self.poleVector_ctrl = CtrlCreate(self.name+"_IK_pole_Ctrl",matrix=self.get_pole_matrix(),ctrl_type="sphere")
        self.handle_ctrl.ctrl.addAttr("follow",k=1,at="enum",en="master:")
        self.handle_ctrl.ctrl.addAttr("poleVector",k=1,min=0,max=1)
        self.handle_ctrl.ctrl.addAttr("twist",k=1)
        self.handle_ctrl.ctrl.addAttr("autoStretch",k=1,min=0,max=1)
        self.handle_ctrl.ctrl.addAttr("manualStretch",k=1)


        


    def jnt_bind(self):
        handle = pm.ikHandle(sj=self.jnts[0],ee=self.jnts[-1],n=self.name + "_IKHandle")[0]
        pm.parent(handle,self.handle_ctrl.ctrl)
        pm.poleVectorConstraint(self.poleVector_ctrl.ctrl,handle)

        streGrp = pm.group(em=1,p=self.jnts[0].getParent(),name=self.name + "_stretchGrp")
        streLoc_1 = pm.group(em=1,p=streGrp,name=self.name + "_stretch_%s_loc"%self.jnts[0])
        streLoc_1.setMatrix(self.jnts[0].getMatrix(ws=1),ws=1)
        pm.parentConstraint(self.jnts[0].getParent(),streLoc_1,mo=1)
        streLoc_2 = pm.group(em=1,p=streGrp,name=self.name + "_stretch_%s_loc"%self.jnts[0])
        pm.parentConstraint(self.handle_ctrl.ctrl,streLoc_2,mo=0)

        tmp_dist = quickCreateNode(type="distanceBetween",point1=streLoc_1.translate,point2=streLoc_2.translate)
        tmp_mult = quickCreateNode(type="multiplyDivide",input1X=tmp_dist.distance,input2X=tmp_dist.distance.get(),operation=2)

        tmp_md = quickCreateNode(type="multDoubleLinear",input1=999,input2 = self.handle_ctrl.ctrl.autoStretch)
        tmp_ad = quickCreateNode(type="addDoubleLinear",input1=1,input2=tmp_md.output)
        tmp_clamp = quickCreateNode(type="clamp",minR=1,maxR=tmp_ad.output,inputR=tmp_mult.outputX)

        tmp_md = quickCreateNode(type="multDoubleLinear",input1=tmp_clamp.outputR,input2=self.jnts[1].tx.get())
        tmp_ad = quickCreateNode(type="addDoubleLinear",input1=self.handle_ctrl.ctrl.manualStretch,input2=tmp_md.output)
        tmp_ad.output.connect(self.jnts[1].tx,f=1)

        tmp_md = quickCreateNode(type="multDoubleLinear",input1=tmp_clamp.outputR,input2=self.jnts[2].tx.get())
        tmp_ad = quickCreateNode(type="addDoubleLinear",input1=self.handle_ctrl.ctrl.manualStretch,input2=tmp_md.output)
        tmp_ad.output.connect(self.jnts[2].tx,f=1)



    
        #return handle

    def build(self):
        self.create_ctrls()
        self.jnt_bind()
    
    def get_groups(self):
        return self.handle_ctrl.offset,self.poleVector_ctrl.offset

class chian_FK():

    def __init__(self,jnts,name):
        self.name=name
        self.jnts = jnts
        self.ctrls = []
        
    def create_ctrls(self):
        for jnt in self.jnts[:-1]:
            ctrl = CtrlCreate(jnt.name() + "_Ctrl",matrix = jnt.getMatrix(ws=1))
            self.ctrls.append(ctrl)
        for i in range(len(self.ctrls))[1:][::-1]:
            pm.parent(self.ctrls[i].offset,self.ctrls[i-1].ctrl)
    
    def jnt_bind(self):

        for ctrl,jnt in zip(self.ctrls,self.jnts[:-1]):
            pm.parentConstraint(ctrl.dummy,jnt,mo=0)

    @property
    def ctrl_offsets(self):
        return [self.ctrls[0].offset]

    def build(self):
        self.create_ctrls()
        self.jnt_bind()

    def get_groups(self):
        return [ctrl.offset for ctrl in self.ctrls]


    

class blend_IK_FK():

    def __init__(self,jnts,name):
        self.jnts = jnts
        self.name = name
        self.setting_ctrl = None
        self.ik_rig = None
        self.fk_rig = None
        self.bend_ctrls = []

        set_ctrl_scale(jnts[1].translate.get().length() / 5.0)
        
        


    def duplicate_jnts(self,jnts,name):

        copy_jnt = jnts[0].duplicate(name=jnts[0].name() +name )[0]
        allChildren = copy_jnt.listRelatives(ad=1)
        notJntlist = []
        for c in allChildren:
            if c.type()=="joint":
                c.rename(c.name() + name)
            else:
                notJntlist.append(c)
        
        if notJntlist:
            [allChildren.remove(i) for i in notJntlist]
            pm.delete(notJntlist)
        reslut_jnt = allChildren[::-1]
        reslut_jnt.insert(0,copy_jnt)
        return reslut_jnt

    def blend_IKFK(self,baseJnts,ikJnts,fkJnts):
        for b,i,f in zip(baseJnts,ikJnts,fkJnts):
            tbc = quickCreateNode(type="blendColors",color1=f.translate,color2=i.translate,blender=self.setting_ctrl.ctrl.IKFK)
            tbc.output.connect(b.translate)
            rbc = quickCreateNode(type="blendColors",color1=f.rotate,color2=i.rotate,blender=self.setting_ctrl.ctrl.IKFK)
            rbc.output.connect(b.rotate)

            [self.setting_ctrl.ctrl.IKFK.connect(c.v,f=1) for c in self.fk_rig.ctrl_offsets]
            reverseNode = quickCreateNode(type="reverse",inputX=self.setting_ctrl.ctrl.IKFK)
            [reverseNode.outputX.connect(c.v,f=1) for c in self.ik_rig.ctrl_offsets]


    def create_ctrls(self):
        ctrl_matrix = pm.dt.Matrix()
        pos = self.jnts[0].getTranslation(space="world") * 0.5 + self.jnts[1].getTranslation(space="world")
        ctrl_matrix[-1] = [pos[0],pos[1],pos[2],1]
        self.setting_ctrl = CtrlCreate(self.name + "IKFK_setting",matrix=ctrl_matrix,ctrl_type="hex")
        self.setting_ctrl.ctrl.addAttr("IKFK",min=0,max=1,dv=0,k=1)
        
        tmp_matrix = self.jnts[0].getMatrix(ws=1)
        pos = self.jnts[0].getTranslation(space="world") * 0.5 + self.jnts[1].getTranslation(space="world") * 0.5
        tmp_matrix[-1] = [pos[0],pos[1],pos[2],1]
        bend_ctrl_1 = CtrlCreate(self.jnts[0].name() + "_mid_Ctrl",matrix=tmp_matrix,ctrl_type="hex")
        tmp_matrix = self.jnts[1].getMatrix(ws=1)
        bend_ctrl_2 = CtrlCreate(self.jnts[1].name() + "_Ctrl",matrix=tmp_matrix,ctrl_type="hex")
        pos = self.jnts[1].getTranslation(space="world") * 0.5 + self.jnts[2].getTranslation(space="world") * 0.5
        tmp_matrix[-1] = [pos[0],pos[1],pos[2],1]
        bend_ctrl_3 = CtrlCreate(self.jnts[1].name() + "_mid_Ctrl",matrix=tmp_matrix,ctrl_type="hex")
        self.bend_ctrls = [bend_ctrl_1,bend_ctrl_2,bend_ctrl_3]
        
        pm.pointConstraint(self.jnts[0],bend_ctrl_2.ctrl,bend_ctrl_1.offset,mo=1)
        #pm.orientConstraint(self.jnts[0],bend_ctrl_1.offset,mo=1)
        pm.pointConstraint(self.jnts[1],bend_ctrl_2.offset,mo=1)

        pm.orientConstraint(self.jnts[0],self.jnts[1],bend_ctrl_2.offset,mo=1)
        
        pm.pointConstraint(bend_ctrl_2.ctrl,self.jnts[-1],bend_ctrl_3.offset)
    

    def build(self):
        ikJnts = self.duplicate_jnts(self.jnts,"_IK")
        fkJnts = self.duplicate_jnts(self.jnts,"_FK")
        for i in range(len(ikJnts))[1:][::-1]:
            pm.parent(ikJnts[i],ikJnts[i-1])
        for i in range(len(fkJnts))[1:][::-1]:
            pm.parent(fkJnts[i],fkJnts[i-1])
        self.fk_rig = chian_FK(fkJnts,self.name)
        self.fk_rig.build()
        self.ik_rig = three_jnt_Ik(ikJnts,self.name)
        self.ik_rig.build()
        self.blend_IKFK(self.jnts,ikJnts,fkJnts)
        self.create_bend(*self.jnts[:-1])
        self.create_bend(*self.jnts[1:])
        
    def create_bend(self,sJnt,eJnt):
        bendCtrl = self.bend_ctrls[0 if self.jnts.index(sJnt)==0 else 2]
        bend_ik_jnt_1 = sJnt.duplicate(po=1,name=sJnt.name() + "_Bend_IK")[0]
        bend_ik_jnt_2 = eJnt.duplicate(po=1,name=eJnt.name() + "_Bend_IK")[0]
        pm.parent(bend_ik_jnt_2,bend_ik_jnt_1)
        pm.orientConstraint(bend_ik_jnt_1,bendCtrl.offset,mo=1)
        

        
        base_point = sJnt.getTranslation(space="world")
        base_dir = eJnt.getTranslation(space="world") - base_point
        curve_points = [base_point]
        for i in range(1,4):
            parmu = i * 0.25
            curve_points.append(base_point + base_dir * parmu)
        curve_points.append(eJnt.getTranslation(space="world"))
        bend_curve = pm.curve(p=curve_points,d=3,name=sJnt.name() + "_BendCurve")
        pm.parent(bend_curve,sJnt)
        bend_ikHandle = pm.ikHandle(sj=bend_ik_jnt_1,ee=bend_ik_jnt_2,n=sJnt.name() + "_Bend_IKHandle")[0]

        twis_pxy_wuo_off = pm.group(em=1,p=self.jnts[0].getParent(),name=sJnt.name() + "_TwisWUO_offset")
        twis_pxy_wuo = pm.group(em=1,p=twis_pxy_wuo_off,name=sJnt.name() + "_TwisWUO")
        twis_pxy_wuo_off.setMatrix(eJnt.getMatrix(ws=1),ws=1)
        twis_pxy_wuo_off.setTranslation(twis_pxy_wuo_off.getTranslation(space="world") + pm.dt.Vector(eJnt.getMatrix(ws=1)[1][:-1]) ,space="world")
        
        pm.dt.Vector(eJnt.getMatrix(ws=1)[1][:-1])
        pm.parentConstraint(sJnt,twis_pxy_wuo_off,mo=1)
        twis_pxy_jnt = bend_ik_jnt_2.duplicate(po=1,name=sJnt.name()+ "_TwisPxyJnt")[0]

        pm.parentConstraint(eJnt,twis_pxy_jnt,skipRotate=["x","y","z"],mo=1)
        pm.aimConstraint(sJnt,twis_pxy_jnt,aim=[-1,0,0],wu=[0,1,0],wuo=twis_pxy_wuo,mo=1,wut="object")

        pm.parent(bend_ikHandle,bend_ik_jnt_1.getParent())
        if self.jnts.index(sJnt) == 0:
            pm.pointConstraint(self.bend_ctrls[1].ctrl,bend_ikHandle,mo=1)
        else:
            pm.pointConstraint(eJnt,bend_ikHandle,mo=1)
            pm.pointConstraint(self.bend_ctrls[1].ctrl,bend_ik_jnt_1,mo=1)

        # if self.jnts.index(eJnt) == 1:
        #     pm.parentConstraint(bend_ikHandle,self.bend_ctrls[1].ctrl)

        bend_jnt_list = []
        
        for i in range(4):
            bend_jnt = sJnt.duplicate(po=1,name=sJnt.name() + "_Bend_Jnt_%s"%i)[0]
            pm.parent(bend_jnt,bend_ik_jnt_1)
            parmu = i * 0.25
            mp = quickCreateNode(type="motionPath",uValue=parmu,fractionMode=1,geometryPath=bend_curve.worldSpace)
            compMatrix = quickCreateNode(type="composeMatrix",inputTranslate=mp.allCoordinates)
            multMatrix = quickCreateNode(type="multMatrix")
            compMatrix.outputMatrix.connect(multMatrix.matrixIn[0])
            sJnt.worldInverseMatrix.connect(multMatrix.matrixIn[1])
            decompMatrix = quickCreateNode(type="decomposeMatrix",inputMatrix=multMatrix.matrixSum)
            decompMatrix.outputTranslate.connect(bend_jnt.translate)
            bend_jnt_list.append(bend_jnt)
            md = quickCreateNode(type="multDoubleLinear",input1=twis_pxy_jnt.rx,input2=parmu)
            ad = quickCreateNode(type="addDoubleLinear",input1=md.output,input2=bendCtrl.ctrl.rx)
            ad.output.connect(bend_jnt.rx)

        #pm.select(cl=1)
        curve_sk_jnt_1 = sJnt.duplicate(po=1,name=bend_curve.name() + "_skJnt_1")[0]
        print(bend_jnt_list[2].translate.get())
        curve_sk_jnt_2 = bend_jnt_list[2].duplicate(po=1,name=bend_curve.name() + "_skJnt_2")[0]
        curve_sk_jnt_3 = bend_jnt_list[2].duplicate(po=1,name=bend_curve.name() + "_skJnt_3")[0]
        pm.parent(curve_sk_jnt_1,w=1)
        pm.parent(curve_sk_jnt_2,curve_sk_jnt_1)
        pm.parent(curve_sk_jnt_3,curve_sk_jnt_2)

        if self.jnts.index(sJnt) == 0:
            sLoc_1 = pm.group(em=1,p=sJnt,name=curve_sk_jnt_1.name() + "_scale_loc")
            sLoc_2 = pm.group(em=1,p=sJnt,name=curve_sk_jnt_1.name() + "_scale_loc")
            pm.pointConstraint(self.bend_ctrls[1].ctrl,sLoc_2,mo=0)
        else:
            sLoc_1 = pm.group(em=1,p=eJnt,name=curve_sk_jnt_1.name() + "_scale_loc")
            sLoc_2 = pm.group(em=1,p=eJnt,name=curve_sk_jnt_1.name() + "_scale_loc")
            pm.pointConstraint(self.bend_ctrls[1].ctrl,sLoc_1,mo=0)
        
        tmp_dist = quickCreateNode(type="distanceBetween",point1=sLoc_1.translate,point2=sLoc_2.translate)
        tmp_mult = quickCreateNode(type="multiplyDivide",input1X=tmp_dist.distance,input2X=tmp_dist.distance.get(),operation=2)
        tmp_mult.outputX.connect(curve_sk_jnt_1.scaleX)
        
        sk = pm.skinCluster([curve_sk_jnt_1,curve_sk_jnt_3],bend_curve,name=bend_curve.name()+"_SKIN",tsb=1)
        sk.setWeights(bend_curve,[0,1],[1,0,0,1,0,1,0,1,1,0])

        self.bend_ctrls[0 if self.jnts.index(sJnt)==0 else 2].ctrl.translate.connect(curve_sk_jnt_3.translate)


            



if __name__ =="__main__":
    a = blend_IK_FK(pm.selected(),"testArm")
    a.create_ctrls()
    a.build()
