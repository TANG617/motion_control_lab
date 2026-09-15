"""Independent XML geometric Jacobians; no native solver/FK imports."""
import numpy as np
from inputs import UrdfFk

class GeometricFk(UrdfFk):
    def spatial_jacobian(self,names,q,frame,tcp=(0,0,0),root='base_link'):
        frames=self._frames(names,q)
        point=frames[frame][:3,3]+frames[frame][:3,:3]@np.asarray(tcp)
        parent_by_child={j[2]:j for j in self._compiled};ancestors=[];child=frame
        while child in parent_by_child:
            j=parent_by_child[child];ancestors.append(j);child=j[1]
        result=np.zeros((6,len(names)));columns={n:i for i,n in enumerate(names)}
        for name,parent,child,origin,axis,kind,mimic in ancestors:
            if kind=='fixed':continue
            factor=1.
            if mimic is not None:name,factor,_=mimic
            if name not in columns:continue
            joint=frames[parent]@origin;a=joint[:3,:3]@np.asarray(axis)
            if kind in ('revolute','continuous'):
                result[:3,columns[name]]+=factor*np.cross(a,point-joint[:3,3]);result[3:,columns[name]]+=factor*a
            elif kind=='prismatic':result[:3,columns[name]]+=factor*a
            else:raise ValueError('unsupported geometric joint type '+kind)
        rotation=frames[root][:3,:3].T
        result[:3]=rotation@result[:3];result[3:]=rotation@result[3:]
        return result

    def jacobian(self,names,q,frame,tcp=(0,0,0),eps=None):
        return self.spatial_jacobian(names,q,frame,tcp)[:3]

    def angular_jacobian(self,names,q,frame):
        return self.spatial_jacobian(names,q,frame)[3:]

    def finite_difference(self,names,q,frame,tcp=(0,0,0),eps=1e-6):
        return UrdfFk.jacobian(self,names,q,frame,tcp,eps)
