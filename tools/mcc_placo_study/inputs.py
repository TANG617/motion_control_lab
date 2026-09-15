"""Canonical input and independent URDF FK. Does not import candidate solvers."""
import functools, math, pathlib, xml.etree.ElementTree as ET
import numpy as np
from evidence import artifact, stable_hash, write_json
JOINTS=['head_yaw_joint','head_pitch_joint','torso_yaw_joint','torso_pitch_joint','knee_pitch_joint','ankle_pitch_joint']+[f'{s}_arm_joint{i}' for s in ('left','right') for i in range(1,8)]
ACTIVE=[j for j in JOINTS if j not in ('head_yaw_joint','head_pitch_joint','knee_pitch_joint','ankle_pitch_joint')]
Q0=[0,.305,0,.289,.05,-.05,.925,-1.448,-1.483,-1.401,-1.175,0,0,-.925,1.448,1.483,1.401,1.175,0,0]

def rotation(axis,angle):
    a=np.asarray(axis,dtype=float); a=a/np.linalg.norm(a)
    x,y,z=a; k=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3)+math.sin(angle)*k+(1-math.cos(angle))*(k@k)

def xyz(text): return [float(v) for v in text.split()]
class UrdfFk:
    """Fixed-base URDF tree evaluator independent of Pinocchio; mimic joints explicit."""
    def __init__(self,path):
        self.path=pathlib.Path(path); self.root=ET.parse(path).getroot(); self.joints=list(self.root.findall('joint'))
        children={j.find('child').get('link') for j in self.joints}
        self.base=next(l.get('name') for l in self.root.findall('link') if l.get('name') not in children)
        # Compile immutable model data once, preserving the original traversal and
        # floating-point operation order. This is independent XML FK, not candidate FK.
        self._compiled=[];pending=list(self.joints);seen={self.base}
        while pending:
            progressed=False
            for j in pending[:]:
                parent=j.find('parent').get('link')
                if parent not in seen:continue
                t=np.eye(4);origin=j.find('origin')
                if origin is not None:
                    t[:3,3]=xyz(origin.get('xyz','0 0 0'));r,p,y=xyz(origin.get('rpy','0 0 0'))
                    t[:3,:3]=rotation([0,0,1],y)@rotation([0,1,0],p)@rotation([1,0,0],r)
                axis=j.find('axis');axis=xyz(axis.get('xyz','1 0 0')) if axis is not None else [1,0,0]
                mimic=j.find('mimic')
                mimic=None if mimic is None else (mimic.get('joint'),float(mimic.get('multiplier','1')),float(mimic.get('offset','0')))
                child=j.find('child').get('link')
                self._compiled.append((j.get('name'),parent,child,t,axis,j.get('type'),mimic))
                seen.add(child);pending.remove(j);progressed=True
            if not progressed:raise ValueError('disconnected URDF')
        self._cached=functools.lru_cache(maxsize=256)(self._evaluate)

    def _evaluate(self,names,q_bytes):
        positions={j.get('name'):0. for j in self.joints}
        positions.update(zip(names,np.frombuffer(q_bytes,dtype=np.float64)))
        frames={self.base:np.eye(4)}
        for name,parent,child,t,axis,kind,mimic in self._compiled:
            value=positions.get(name,0.)
            if mimic is not None:value=positions[mimic[0]]*mimic[1]+mimic[2]
            motion=np.eye(4)
            if kind in ('revolute','continuous'):motion[:3,:3]=rotation(axis,value)
            elif kind=='prismatic':motion[:3,3]=np.asarray(axis)*value
            elif kind!='fixed':raise ValueError('unsupported independent FK joint '+kind)
            frames[child]=frames[parent]@t@motion
        for frame in frames.values():frame.setflags(write=False)
        return frames

    def _frames(self,names,q):
        # Exact bytes preserve signed zero; no rounding/quantization of states.
        return self._cached(tuple(names),np.asarray(q,dtype=np.float64).tobytes())

    def transforms(self,names,q):
        # Preserve the public method's caller-owned mutable result contract.
        return {name:frame.copy() for name,frame in self._frames(names,q).items()}

    def pose(self,names,q,frame,tcp=(0,0,.1),root='base_link'):
        f=self._frames(names,q);t=np.linalg.inv(f[root])@f[frame];t[:3,3]+=t[:3,:3]@np.asarray(tcp)
        return {'position':t[:3,3].tolist(),'rotation':t[:3,:3].tolist()}
    def limits(self,names):
        joints={j.get('name'):j for j in self.joints}
        return {key:[float(joints[n].find('limit').get(attr)) for n in names] for key,attr in [('lower','lower'),('upper','upper'),('velocity','velocity')]}
    def jacobian(self,names,q,frame,tcp=(0,0,.1),eps=1e-6):
        columns=[]
        for i in range(len(q)):
            a=list(q); b=list(q); a[i]+=eps;b[i]-=eps
            columns.append((np.asarray(self.pose(names,a,frame,tcp)['position'])-self.pose(names,b,frame,tcp)['position'])/(2*eps))
        return np.asarray(columns).T

def canonical_snapshot(model,seed=20260910,perturbation=0.,fixture=False):
    fk=UrdfFk(model); limits=fk.limits(JOINTS); rng=np.random.default_rng(seed); target=np.asarray(Q0,dtype=float)
    rejected=0
    for i,j in enumerate(JOINTS):
        if j in ACTIVE:
            target[i]+=rng.uniform(-perturbation,perturbation)*(limits['upper'][i]-limits['lower'][i])
    if np.any(target<limits['lower']) or np.any(target>limits['upper']): raise ValueError('generated target outside limits; rejected')
    return {'schema_version':'canonical_input.v1','input_id':f'r1-{seed}-{perturbation}',
      'fixture':fixture,'source_kind':'synthetic','seed':seed,'generator':artifact(__file__),
      'generator_parameters':{'perturbation_fraction':perturbation,'rejected_candidates':rejected},
      'model':artifact(model),'geometry_policy':'model-only; geometry comparisons require mesh inventory',
      'joint_names':list(JOINTS),'active_joint_names':list(ACTIVE),'root_frame':'base_link',
      'tcp_offsets':{'left':[0,0,.1],'right':[0,0,.1]},'frames':{s:f'{s}_arm_ee_link' for s in ('left','right')},
      'units':{'q':'rad','v':'rad/s','position':'m','rotation':'matrix3x3'},'limits':limits,
      'initial_state':{'q':list(Q0),'v':[0.]*20},'samples':[{'sequence':0,'source_time_s':0.,'q':list(Q0),'v':[0.]*20,
      'targets':{s:fk.pose(JOINTS,target,f'{s}_arm_ee_link') for s in ('left','right')}}],
      'session_id':'synthetic','split_id':'development','derivative_policy':'explicit-zero-feedback; target derivatives absent'}

def save_input(path,data):
    write_json(path,data)
    descriptor={'schema_version':'input_descriptor.v1','input_id':data['input_id'],'canonical':artifact(path),
                'fixture':data.get('fixture',False),'session_id':data.get('session_id','unknown'),
                'split_id':data.get('split_id','unknown'),'model':data.get('model')}
    target=pathlib.Path(path).with_suffix('.descriptor.json');write_json(target,descriptor)
    return str(target.resolve())


def resolve_input(ref, base):
    """Read and verify an archived input descriptor for offline validation."""
    path = pathlib.Path(ref)
    if not path.is_absolute():
        path = base / path
    from evidence import read_json, verify_artifact
    descriptor = read_json(path)
    if descriptor['schema_version'] != 'input_descriptor.v1':
        raise ValueError('input descriptor version')
    if descriptor.get('canonical') is None:
        raise ValueError(descriptor.get('reason', 'canonical input unavailable'))
    if not verify_artifact(descriptor['canonical']):
        raise ValueError('canonical hash mismatch or missing')
    value = read_json(descriptor['canonical']['locator'])
    if value.get('model') and not verify_artifact(value['model']):
        raise ValueError('model hash mismatch or missing')
    return descriptor, value
