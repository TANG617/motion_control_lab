"""Offline actual-output task costs and independent convex sequential references.

No native solver/app imports. Disabled tasks remain evaluation targets. All costs
use recorded output velocity at each method's own state; no tick independence.
"""
from pathlib import Path
import sys
import numpy as np
from scipy.optimize import minimize, LinearConstraint, Bounds
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[1]/'tools/mcc_placo_study'))
from repair.kinematics import GeometricFk

class CachedGeometry(GeometricFk):
    def _frames(self,names,q):
        key=tuple(q)
        if getattr(self,'_key',None)!=key:
            self._key=key;self._cache=super()._frames(names,q)
        return self._cache

def task_rows(data,c,row,fk):
    """Construct evaluation tasks even when primary-only disabled their solve rows."""
    if row['record_type']=='analytic':
        return [dict(name='analytic-task-'+str(i),A=np.asarray(t['A'],float),b=np.asarray(t['b'],float),weight=t.get('weight',1.),level=i,secondary=i>0,declared=t.get('enabled',True),enabled=t.get('enabled',True) and (not c.get('primary_only') or i==0)) for i,t in enumerate(row['problem']['tasks'])]
    names=data['joint_names'];active=[names.index(n) for n in data['active_joint_names']];q=np.asarray(row['input_state']['q']);result=[];g=c['gain_per_s'];swap=c.get('swap_priorities',False)
    enabled=bool(row.get('secondary_enabled',True)) and not c.get('primary_only')
    for side in ('left','right'):
        frame=data['frames'][side];pose=fk.pose(names,q,frame,tcp=(0,0,0));target=row['targets'][side]
        goal=np.asarray(target['position'])-np.asarray(target['rotation'])@data['tcp_offsets'][side]
        result.append(dict(name=side+'-position',A=fk.jacobian(names,q,frame)[:,active],b=g*(goal-pose['position']),weight=1.,level=1 if swap else 0,secondary=False,declared=True,enabled=True,enforcement=c.get('enforcement','soft')))
        result.append(dict(name=side+'-orientation',A=fk.angular_jacobian(names,q,frame)[:,active],b=g*Rotation.from_matrix(np.asarray(target['rotation'])@np.asarray(pose['rotation']).T).as_rotvec(),weight=c['orientation_weight'],level=0 if swap else 1,secondary=True,declared=True,enabled=not c.get('primary_only')))
        if side in row.get('elbow_targets',{}):
            frame=side+'_arm_link4';pose=fk.pose(names,q,frame,tcp=(0,0,0))
            result.append(dict(name=side+'-elbow',A=fk.jacobian(names,q,frame)[:,active],b=g*(np.asarray(row['elbow_targets'][side])-pose['position']),weight=c['secondary_weight'],level=2,secondary=True,declared=bool(c.get('secondary_tasks') and c.get('secondary_link4')),enabled=enabled and c.get('secondary_tasks') and c.get('secondary_link4')))
    if row.get('posture_target'):
        result.append(dict(name='posture',A=np.eye(len(active)),b=g*(np.asarray(row['posture_target'])[active]-q[active]),weight=c['secondary_weight'],level=2,secondary=True,declared=bool(c.get('secondary_tasks') and c.get('secondary_posture')),enabled=enabled and c.get('secondary_tasks') and c.get('secondary_posture')))
    return result

def output_vector(data,row):
    if row['record_type']=='analytic':
        return np.asarray(row['candidate'],float) if row.get('accepted') and row.get('candidate') else None
    active=[data['joint_names'].index(n) for n in data['active_joint_names']]
    v=np.asarray(row.get('v',[]),float)
    return v[active] if len(v)==len(data['joint_names']) and np.all(np.isfinite(v)) else None

def costs(tasks,x):
    return [dict(task=t['name'],secondary=t['secondary'],declared=t.get('declared',True),solve_enabled=t['enabled'],weight=t['weight'],raw_cost=None if x is None or not t.get('declared',True) else float(.5*np.sum((t['A']@x-t['b'])**2)),weighted_cost=None if x is None or not t.get('declared',True) else float(.5*t['weight']*np.sum((t['A']@x-t['b'])**2))) for t in tasks]

def velocity_bounds(data,c,row):
    if row['record_type']=='analytic':return np.asarray(row['problem']['lower'],float),np.asarray(row['problem']['upper'],float)
    active=[data['joint_names'].index(n) for n in data['active_joint_names']];q=np.asarray(row['input_state']['q'])[active];v=np.asarray(row['input_state']['v'])[active];dt=c['period_s'];l=np.asarray(data['limits']['lower'])[active];h=np.asarray(data['limits']['upper'])[active];vm=np.asarray(data['limits']['velocity'])[active]
    lo=np.maximum((l-q)/dt,-vm);hi=np.minimum((h-q)/dt,vm)
    if c.get('native_acceleration'):
        a=c['acceleration_limit'];lo=np.maximum.reduce([lo,v-a*dt,-np.sqrt(np.maximum(2*a*(q-l),0))]);hi=np.minimum.reduce([hi,v+a*dt,np.sqrt(np.maximum(2*a*(h-q),0))])
    return lo,hi

def sequential_reference(tasks,lower,upper,regularization,preservation_tolerance,hard_tolerance,method,maxiter=200):
    """Convex QP per configured level. Certify feasibility and NNLS KKT stationarity.

    Solver success alone is insufficient. Rank-deficient optima compare objective
    values, never equality of the reference and native joint vectors.
    """
    if np.any(lower>upper):return dict(status='unavailable',reason='empty original hard set',levels=[])
    enabled=[dict(t) for t in tasks if t['enabled'] and t.get('declared',True)]
    band=.5*preservation_tolerance  # native current-source policy; old binary binding unavailable
    hard=[t for t in enabled if t.get('enforcement')=='hard']
    active=[t for t in enabled if t.get('enforcement')!='hard']
    hierarchy='hqp' in method
    for t in active:
        if not hierarchy:t['level']=0
        elif '3' not in method and t['level']==2:t['level']=1
    n=len(lower);x=np.minimum(np.maximum(np.zeros(n),lower),upper);E=np.empty((0,n));f=np.empty(0);levels=[]
    hardA=np.concatenate([t['A'] for t in hard]) if hard else np.empty((0,n));hardb=np.concatenate([t['b'] for t in hard]) if hard else np.empty(0)
    for level in sorted({t['level'] for t in active} or ({0} if hard else set())):
        group=[t for t in active if t['level']==level]
        A=np.concatenate([np.sqrt(t['weight'])*t['A'] for t in group]) if group else np.empty((0,n));b=np.concatenate([np.sqrt(t['weight'])*t['b'] for t in group]) if group else np.empty(0);H=A.T@A+regularization*np.eye(n);g=-A.T@b
        G=np.r_[np.eye(n),-np.eye(n),hardA,-hardA,E,-E];h=np.r_[upper,-lower,hardb,-hardb,f+band,-f+band]
        constraints=[] if not len(E) else [LinearConstraint(E,f-band,f+band)]
        if len(hardA):constraints.append(LinearConstraint(hardA,hardb,hardb))
        r=minimize(lambda z:.5*np.sum((A@z-b)**2)+.5*regularization*np.sum(z*z),x,jac=lambda z:H@z+g,method='SLSQP',bounds=Bounds(lower,upper),constraints=constraints,options={'ftol':1e-12,'maxiter':maxiter})
        slack=h-G@r.x;primal=float(max(0.,-np.min(slack)));grad=H@r.x+g;binding=G[slack<=hard_tolerance]
        dual_res=float(np.linalg.norm(grad,np.inf));complementarity=0.;multipliers=[]
        if len(binding):
            # NNLS stationarity certificate; bounded reference optimizer only.
            from scipy.optimize import lsq_linear
            dual=lsq_linear(binding.T,-grad,bounds=(0,np.inf),tol=1e-12,lsmr_tol=1e-12,max_iter=200)
            dual_res=float(np.linalg.norm(binding.T@dual.x+grad,np.inf));multipliers=dual.x.tolist()
            complementarity=float(np.max(np.abs(dual.x*slack[slack<=hard_tolerance]),initial=0.))
        certificate=bool(r.success and primal<=hard_tolerance and dual_res<=1e-6 and complementarity<=1e-9)
        levels.append(dict(level=level,status='certified' if certificate else 'unavailable',optimizer_success=bool(r.success),optimizer_message=str(r.message),primal_violation=primal,kkt_stationarity_inf=dual_res,kkt_stationarity_tolerance=1e-6,kkt_complementarity_max=complementarity,kkt_complementarity_tolerance=1e-9,kkt_nonnegative_multipliers=multipliers,preservation_band_fraction=.5,preservation_half_width=band,regularization_center='zero primary velocity on each pass',objective=float(.5*np.sum((A@r.x-b)**2)),regularized_objective=float(r.fun),x=r.x.tolist(),tasks=[t['name'] for t in group],A=A.tolist(),b=b.tolist()))
        if not certificate:return dict(status='unavailable',reason='reference optimizer or independent KKT certificate incomplete',levels=levels)
        x=r.x
        # Preserve physical task residuals, not weighted coordinate values.
        if group:
            physical=np.concatenate([t['A'] for t in group]);E=np.r_[E,physical];f=np.r_[f,physical@x]
    return dict(status='certified' if levels else 'unavailable',reason='' if levels else 'no enabled tasks',levels=levels)

def reference_selected(data,c,row):
    if data.get('fixture'):return False
    if c.get('state_mode')=='snapshot':return True
    t=row.get('source_time_s');dt=c['period_s'];switches=[];previous=None
    for sample in data.get('samples',[]):
        identity=(sample.get('task_revision'),sample.get('secondary_enabled'))
        if identity!=previous:switches.append(sample['source_time_s'])
        previous=identity
    return t is not None and any(abs(t-s)<=dt*1e-9 for s in switches)
