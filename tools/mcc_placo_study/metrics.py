"""study-metrics.v1: pure per-sample/per-unit definitions; no cross-run statistics."""
import math
import numpy as np
VERSION='study-metrics.v1'

def position_error(reference,actual): return float(np.linalg.norm(np.asarray(reference)-actual))
def orientation_error(reference,actual):
    r=np.asarray(reference).reshape(3,3).T@np.asarray(actual).reshape(3,3)
    return math.acos(float(np.clip((np.trace(r)-1)/2,-1,1)))
def hard_violation(value,lower,upper): return np.maximum(np.maximum(np.asarray(lower)-value,np.asarray(value)-upper),0)
def hard_violation_excess(value,lower,upper,tolerance): return np.maximum(hard_violation(value,lower,upper)-tolerance,0)
def preservation_ratio(at_level,final,tolerance):
    t=np.asarray(tolerance)
    if np.any(t<=0): raise ValueError('positive declared preservation tolerance required')
    return float(np.max(np.abs(np.asarray(final)-at_level)/t))
def secondary_gain(primary,secondary):
    return {'absolute':primary-secondary,'relative':None if primary==0 else (primary-secondary)/primary,'status':'not-applicable' if primary==0 else 'ok'}
def nearest_rank(values,p):
    if not values: return None
    if not 0<p<=1: raise ValueError('quantile range')
    return sorted(values)[math.ceil(p*len(values))-1]
def window_for_call(index,warmup=100):
    return 'cold' if index==0 else ('warm-up' if index<=warmup else 'steady')
def deadlines(rows):
    misses=[r.get('skipped',False) or r.get('finish_ns') is None or r['finish_ns']>r['deadline_ns'] for r in rows]
    longest=current=0
    for miss in misses:
        current=current+1 if miss else 0; longest=max(longest,current)
    return {'scheduled':len(rows),'misses':sum(misses),'miss_rate':sum(misses)/len(rows) if rows else None,'consecutive_misses':longest}
def paired_admission(required,actual):
    return [{'identity':k,'status':'available' if all(actual.get((k,m))=='completed' for m in methods) else 'unavailable',
             'missing_methods':[m for m in methods if actual.get((k,m))!='completed']} for k,methods in required.items()]
def time_weighted(values,times,end,threshold):
    if len(values)!=len(times) or not values: raise ValueError('sample/time mismatch')
    w=np.diff(list(times)+[end]); a=np.asarray(values)
    if np.any(w<=0): raise ValueError('nonpositive holding interval')
    order=np.argsort(a); cumulative=np.cumsum(w[order]); q=a[order[np.searchsorted(cumulative,.95*sum(w))]]
    return {'rms':float(np.sqrt(sum(w*a*a)/sum(w))),'p95':float(q),'max':float(max(a)),'above_threshold_s':float(sum(w[a>threshold]))}

def event_response_lag(times,values,event_time,baseline,target,fraction=0.5):
    """First observed directional fraction crossing after an a-priori event; no interpolation.
    Positive lag means response after the event. Right-censored when absent.
    """
    if not 0<fraction<=1 or len(times)!=len(values):raise ValueError('event estimator declaration')
    delta=target-baseline
    if delta==0:return {'value':None,'status':'not-applicable','reason':'zero event displacement'}
    for t,v in zip(times,values):
        if t>=event_time and (v-baseline)/delta>=fraction:return {'value':float(t-event_time),'status':'ok','unit':'s','estimator':'first-observed-fraction-crossing','fraction':fraction}
    return {'value':None,'status':'unavailable','reason':'right-censored','observed_after_event_s':max(0,times[-1]-event_time) if times else 0}

def correlation_lag(reference,actual,dt,max_lag_s):
    """Declared uniform-grid overlap Pearson correlation; positive means actual lags.
    Diagnostic only: NEVER shifts the primary simultaneous tracking errors.
    Ties choose smallest absolute lag then smallest signed lag.
    """
    x=np.asarray(reference,float);y=np.asarray(actual,float)
    if dt<=0 or max_lag_s<0 or x.ndim!=1 or y.shape!=x.shape:raise ValueError('lag estimator input/declaration')
    if len(x)<3 or not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)):return {'value':None,'status':'unavailable','reason':'insufficient/nonfinite samples'}
    limit=min(int(math.floor(max_lag_s/dt)),len(x)-3);scores=[]
    for shift in range(-limit,limit+1):
        a=x[:len(x)-shift] if shift>=0 else x[-shift:]
        b=y[shift:] if shift>=0 else y[:len(y)+shift]
        if np.all(a==a[0]) or np.all(b==b[0]):continue
        a=a-np.mean(a);b=b-np.mean(b);denom=np.linalg.norm(a)*np.linalg.norm(b)
        if denom>0:scores.append((float(a@b/denom),shift))
    if not scores:return {'value':None,'status':'not-applicable','reason':'zero variance'}
    score,shift=max(scores,key=lambda item:(item[0],-abs(item[1]),-item[1]))
    return {'value':shift*dt,'unit':'s','status':'ok','correlation':score,'estimator':'overlap-pearson-uniform-grid',
            'dt_s':dt,'max_lag_s':max_lag_s,'primary_errors_shifted':False}

def directional_progress(reference_start,reference_goal,actual_start,actual):
    direction=np.asarray(reference_goal,float)-reference_start;norm=float(np.linalg.norm(direction))
    if norm==0:return {'value':None,'status':'not-applicable','reason':'zero declared direction'}
    return {'value':float((np.asarray(actual)-actual_start)@direction/norm),'status':'ok','unit':'m','definition':'displacement projected on preregistered unit direction'}

def error_reduction(initial_error,current_error):return float(initial_error-current_error)

def stall_duration(times,progress_values,speed_threshold):
    """Longest/total observed interval with directional progress rate <= threshold.
    Rates are interval differences; the last sample has no invented holding duration.
    """
    t=np.asarray(times,float);p=np.asarray(progress_values,float)
    if len(t)!=len(p) or len(t)<2:return {'value':None,'status':'unavailable','reason':'fewer than two observed progress samples'}
    durations=np.diff(t)
    if np.any(durations<=0) or speed_threshold<0:raise ValueError('stall time/threshold declaration')
    rates=np.diff(p)/durations;longest=current=total=0.
    for dt,rate in zip(durations,rates):
        current=current+dt if rate<=speed_threshold else 0.;longest=max(longest,current)
        if rate<=speed_threshold:total+=dt
    return {'value':float(longest),'total_s':float(total),'unit':'s','status':'ok','threshold_mps':speed_threshold,
            'observed_duration_s':float(t[-1]-t[0]),'rate_policy':'observed-interval-forward-difference; last boundary excluded'}

def recovery_time(times,position_errors,orientation_errors,accepted,released_at,position_threshold,orientation_threshold,dwell_s,observation_end):
    """First onset of a continuously good held-sample dwell after release; censored otherwise."""
    if not (len(times)==len(position_errors)==len(orientation_errors)==len(accepted)):raise ValueError('recovery sample mismatch')
    if dwell_s<=0 or min(position_threshold,orientation_threshold)<0:raise ValueError('recovery threshold declaration')
    onset=None
    for i,t in enumerate(times):
        end=times[i+1] if i+1<len(times) else observation_end
        if end<=t:raise ValueError('nonpositive recovery holding interval')
        if end<=released_at:continue
        start=max(t,released_at)
        good=bool(accepted[i]) and np.all(np.asarray(position_errors[i])<=position_threshold) and np.all(np.asarray(orientation_errors[i])<=orientation_threshold)
        if not good:onset=None;continue
        if onset is None:onset=start
        if end-onset>=dwell_s:
            return {'value':float(onset-released_at),'status':'ok','unit':'s','confirmed_at_s':float(onset+dwell_s),'dwell_s':dwell_s,'censored':False}
    return {'value':None,'status':'unavailable','reason':'not-recovered-within-observation','censored':True,
            'observed_after_release_s':max(0.,observation_end-released_at),'dwell_s':dwell_s}
