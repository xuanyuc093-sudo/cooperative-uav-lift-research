"""Reduced 3-D cooperative transport model, NOT a full multirotor simulator.

Rigid payload (6 DoF), three point-mass vehicles, tension-only elastic massless
cables, and first-order vector-force actuators. Vehicle attitude, rotor mixing,
rotor wakes, ground contact and inelastic cable-impact laws are NOT included.
All default physical and controller parameters are research assumptions.
"""
from dataclasses import dataclass, asdict
from pathlib import Path
import math
import json
import numpy as np


def cross(a,b):
    return np.cross(a,b)

def skew(r):
    x,y,z=r
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])

def rotation(q):
    w,x,y,z=q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def quat_derivative(q,w):
    qw=q[0];qv=q[1:]
    return .5*np.r_[-qv@w,qw*w+cross(qv,w)]

def vee(s):
    return np.array([s[2,1],s[0,2],s[1,0]])


@dataclass
class Config:
    topology: str = 'distributed'
    controller: str = 'cooperative'
    vehicle_mass: float = .6
    payload_mass: float = .6
    cable_length: float = 1.
    inclination_deg: float = 20.
    attachment_radius: float = .15
    attachment_height: float = .05
    stiffness: float = 1000.
    damping: float = 1.
    force_cap: float = 12.8
    tilt_deg: float = 25.
    actuator_tau: float = .04
    gravity: float = 9.81
    density: float = 1.225
    drag_vehicle: float = .02
    drag_payload: float = .09
    wind: float = 0.
    command_delay: float = 0.
    payload_mass_error: float = 0.
    speed: float = 1.
    acceleration: float = .5
    jerk: float = 1.
    yaw_target_deg: float = 0.
    initial_roll_deg: float = 0.
    dt: float = .002
    control_dt: float = .01
    seed: int = 0
    log_dt: float = .02


class Motion:
    """Analytic symmetric seven-segment jerk-limited rest-to-rest translation."""
    def __init__(self,v,a,j,cruise=2.,minimum_distance=4.,start=2.):
        if min(v,a,j)<=0:raise ValueError('Motion limits must be positive')
        self.v=v;self.j=j;self.start=start
        if v>=a*a/j:
            tj=a/j;ta=v/a-tj
        else:
            tj=math.sqrt(v/j);ta=0.
        self.a_peak=j*tj
        tacc=2*tj+ta
        tc=max(cruise,(minimum_distance-v*tacc)/v)
        self.durations=np.array([tj,ta,tj,tc,tj,ta,tj])
        self.jerks=np.array([j,0,-j,0,-j,0,j])
        self.states=[]
        x=vel=acc=t=0.
        for duration,jrk in zip(self.durations,self.jerks):
            self.states.append((t,x,vel,acc))
            x+=vel*duration+.5*acc*duration**2+jrk*duration**3/6
            vel+=acc*duration+.5*jrk*duration**2
            acc+=jrk*duration;t+=duration
        self.distance=x;self.duration=t
        self.stop_time=start+t
        self.total_time=self.stop_time+4.

    def at(self,t):
        t-=self.start
        if t<=0:return np.zeros(3)
        if t>=self.duration:return np.array([self.distance,0.,0.])
        for (t0,x,v,a),dur,j in zip(self.states,self.durations,self.jerks):
            if t<t0+dur:
                h=t-t0
                return np.array([x+v*h+.5*a*h*h+j*h*h*h/6,v+a*h+.5*j*h*h,a+j*h])
        return np.array([self.distance,0.,0.])


class Model:
    # State layout: load xyz,vxyz,qwxyz,body omega; vehicles xyz[3,3],vxyz[3,3],forces[3,3].
    def __init__(self,cfg):
        self.c=cfg
        self.n=3
        self.phi=np.arange(3)*2*np.pi/3
        self.mass=np.full(3,cfg.vehicle_mass)
        a,b,c=.3,.3,.1
        self.J=cfg.payload_mass/12*np.array([b*b+c*c,a*a+c*c,a*a+b*b])
        r=np.array([[cfg.attachment_radius*np.cos(v),cfg.attachment_radius*np.sin(v),cfg.attachment_height] for v in self.phi])
        if cfg.topology=='common':r[:]=0
        elif cfg.topology=='offset_common':r[:]=[0,0,cfg.attachment_height]
        elif cfg.topology!='distributed':raise ValueError('unknown topology')
        self.r=r
        theta=math.radians(cfg.inclination_deg)
        self.u0=np.array([[np.sin(theta)*np.cos(v),np.sin(theta)*np.sin(v),np.cos(theta)] for v in self.phi])
        self.G=np.vstack([np.tile(np.eye(3),(1,3)),np.hstack([skew(v) for v in r])])
        self.Ginv=np.linalg.pinv(self.G)
        self.null=np.eye(9)-self.Ginv@self.G
        self.motion=Motion(cfg.speed,cfg.acceleration,cfg.jerk)
        self.wind_phase=np.random.default_rng(cfg.seed).uniform(0,2*np.pi)

    def unpack(self,y):
        return y[:3],y[3:6],y[6:10],y[10:13],y[13:22].reshape(3,3),y[22:31].reshape(3,3),y[31:40].reshape(3,3)

    def initial(self):
        c=self.c;y=np.zeros(40);y[2]=1.;y[6]=1.
        if c.initial_roll_deg:
            angle=math.radians(c.initial_roll_deg)/2;y[6:10]=[math.cos(angle),math.sin(angle),0,0]
        R=rotation(y[6:10])
        tension=c.payload_mass*c.gravity/(3*np.cos(math.radians(c.inclination_deg)))
        # Elastic prestrain is necessary for a mechanically balanced initial state.
        y[13:22]=(y[:3]+self.r@R.T+(c.cable_length+tension/c.stiffness)*self.u0).reshape(-1)
        y[31:40]=(self.mass[:,None]*[0,0,c.gravity]+tension*self.u0).reshape(-1)
        return y

    def cable_forces(self,y):
        x,v,q,w,xi,vi,_=self.unpack(y);R=rotation(q)
        rw=self.r@R.T
        endpoints=x+rw
        velocities=v+cross(np.broadcast_to(R@w,rw.shape),rw)
        delta=xi-endpoints
        d=np.linalg.norm(delta,axis=1)
        u=delta/np.maximum(d[:,None],1e-12)
        extension=d-self.c.cable_length
        rate=np.sum((vi-velocities)*u,axis=1)
        raw=self.c.stiffness*extension+self.c.damping*rate
        t=np.where(extension>0,np.maximum(raw,0),0.)
        return t[:,None]*u,t,u,extension

    def wind_at(self,t):
        if self.c.wind==0:return np.zeros(3)
        ramp=min(max(t/4.,0.),1.)
        return np.array([0.,self.c.wind*ramp*(1+.2*np.sin(2*np.pi*.4*t+self.wind_phase)),0.])

    def drag(self,vel,cdA,wind):
        relative=vel-wind
        return -.5*self.c.density*cdA*np.linalg.norm(relative,axis=-1,keepdims=True)*relative

    def rhs(self,t,y,command):
        c=self.c;x,v,q,w,xi,vi,force=self.unpack(y);R=rotation(q)
        cable,ten,u,extension=self.cable_forces(y)
        wind=self.wind_at(t)
        load_force=np.sum(cable,axis=0)+self.drag(v,c.drag_payload,wind)-[0,0,c.payload_mass*c.gravity]
        torque_body=np.sum(cross(self.r,cable@R),axis=0)
        acceleration_vehicle=(force-cable+self.drag(vi,c.drag_vehicle,wind))/self.mass[:,None]-[0,0,c.gravity]
        out=np.zeros_like(y)
        out[:3]=v;out[3:6]=load_force/c.payload_mass
        out[6:10]=quat_derivative(q,w)
        out[10:13]=(torque_body-cross(w,self.J*w))/self.J
        out[13:22]=vi.reshape(-1);out[22:31]=acceleration_vehicle.reshape(-1)
        out[31:40]=((command-force)/c.actuator_tau).reshape(-1)
        return out

    def step(self,t,y,command,h=None):
        h=self.c.dt if h is None else h
        k1=self.rhs(t,y,command)
        k2=self.rhs(t+h/2,y+h*k1/2,command)
        k3=self.rhs(t+h/2,y+h*k2/2,command)
        k4=self.rhs(t+h,y+h*k3,command)
        yn=y+h/6*(k1+2*k2+2*k3+k4)
        yn[6:10]/=np.linalg.norm(yn[6:10])
        return yn

    def reference(self,t):
        xx,vx,ax=self.motion.at(t)
        x=np.array([xx,0.,1.]);v=np.array([vx,0.,0.]);a=np.array([ax,0.,0.])
        # Smooth yaw maneuver uses the same normalized S-curve, for a topology test only.
        d=max(self.motion.distance,1e-12)
        yaw=math.radians(self.c.yaw_target_deg)*xx/d
        yawrate=math.radians(self.c.yaw_target_deg)*vx/d
        yawacc=math.radians(self.c.yaw_target_deg)*ax/d
        co,si=np.cos(yaw),np.sin(yaw)
        Rd=np.array([[co,-si,0.],[si,co,0.],[0.,0.,1.]])
        return x,v,a,Rd,np.array([0.,0.,yawrate]),np.array([0.,0.,yawacc])

    def cap_force(self,desired):
        f=desired.copy()
        f[:,2]=np.maximum(f[:,2],0.)
        horizontal=np.linalg.norm(f[:,:2],axis=1)
        allowable=f[:,2]*np.tan(math.radians(self.c.tilt_deg))
        scale=np.minimum(1.,allowable/np.maximum(horizontal,1e-12))
        f[:,:2]*=scale[:,None]
        mag=np.linalg.norm(f,axis=1)
        f*=np.minimum(1.,self.c.force_cap/np.maximum(mag,1e-12))[:,None]
        return f

    def command(self,t,y):
        c=self.c;x,v,q,w,xi,vi,force=self.unpack(y);R=rotation(q)
        xr,vr,ar,Rd,wr,alphar=self.reference(t)
        estimated_mass=c.payload_mass*(1+c.payload_mass_error)
        pref=estimated_mass*c.gravity/(3*np.cos(math.radians(c.inclination_deg)))*self.u0
        if c.controller=='independent':
            # Fixed reference formation; identical cable-force feedforward and actuation
            # assumptions as the cooperative controller.
            xdes=xr+self.r@Rd.T+(c.cable_length+np.linalg.norm(pref,axis=1)/c.stiffness)[:,None]*self.u0
            vdes=np.broadcast_to(vr,(3,3))
            residual=0.
        elif c.controller=='cooperative':
            desired_force_world=estimated_mass*(ar+3.*(xr-x)+3.*(vr-v)+[0,0,c.gravity])
            eR=.5*vee(Rd.T@R-R.T@Rd)
            ew=w-R.T@Rd@wr
            desired_torque=-.15*eR-.08*ew+cross(w,self.J*w)+self.J*(R.T@Rd@alphar)
            wrench=np.r_[R.T@desired_force_world,desired_torque]
            h=self.Ginv@wrench+self.null@pref.reshape(-1)
            residual=float(np.linalg.norm(self.G@h-wrench))
            mu=h.reshape(3,3)@R.T
            desired_t=np.linalg.norm(mu,axis=1)
            desired_u=mu/np.maximum(desired_t[:,None],1e-9)
            xdes=x+self.r@R.T+(c.cable_length+desired_t/c.stiffness)[:,None]*desired_u
            vdes=v+cross(np.broadcast_to(R@w,(3,3)),self.r@R.T)
        else:raise ValueError('unknown controller')
        actual_cable,ten,u,extension=self.cable_forces(y)
        desired=self.mass[:,None]*(ar+18.*(xdes-xi)+7.*(vdes-vi)+[0,0,c.gravity])+actual_cable
        capped=self.cap_force(desired)
        clipped=np.linalg.norm(desired-capped,axis=1)>1e-6
        return capped,dict(allocation_residual=residual,clipped=clipped,uncapped=desired)

    def energy(self,y):
        x,v,q,w,xi,vi,f=self.unpack(y)
        _,_,_,extension=self.cable_forces(y)
        kinetic=.5*self.c.payload_mass*(v@v)+.5*np.sum(self.mass[:,None]*vi*vi)+.5*np.sum(self.J*w*w)
        potential=self.c.gravity*(self.c.payload_mass*x[2]+np.sum(self.mass*xi[:,2]))
        elastic=.5*self.c.stiffness*np.sum(np.maximum(extension,0.)**2)
        return float(kinetic+potential+elastic)

    def momentum(self,y):
        x,v,q,w,xi,vi,f=self.unpack(y);R=rotation(q)
        linear=self.c.payload_mass*v+np.sum(self.mass[:,None]*vi,axis=0)
        angular=cross(x,self.c.payload_mass*v)+R@(self.J*w)+np.sum(cross(xi,self.mass[:,None]*vi),axis=0)
        return linear,angular


def run_case(cfg,log_path=None,deadline_epoch=None):
    import time
    model=Model(cfg);y=model.initial()
    ratio=cfg.control_dt/cfg.dt
    if abs(ratio-round(ratio))>1e-8:raise ValueError('control_dt must be an integer multiple of dt')
    control_every=int(round(ratio));log_every=max(1,int(round(cfg.log_dt/cfg.dt)))
    delay_steps=int(round(cfg.command_delay/cfg.control_dt))
    command=y[31:40].reshape(3,3).copy()
    commands=[command.copy() for _ in range(delay_steps)]
    diag={'allocation_residual':0.,'clipped':np.zeros(3,dtype=bool)}
    nsteps=int(math.ceil(model.motion.total_time/cfg.dt))
    history=[];start=time.monotonic()
    error_sq=0.;counter=0
    max_position_error=max_attitude=max_force=max_tension=max_extension=max_residual=0.
    max_load_speed=max_vehicle_speed=max_cable_deviation=max_force_tilt=0.
    min_tension=float('inf');min_distance=float('inf');slack_count=sat_count=0
    completed=True;reason='complete'
    for step in range(nsteps+1):
        t=step*cfg.dt
        if step%100==0 and deadline_epoch is not None and time.time()>=deadline_epoch:
            completed=False;reason='wall_clock_deadline';break
        if step%control_every==0:
            computed,diag=model.command(t,y)
            commands.append(computed)
            command=commands.pop(0) if delay_steps else commands.pop()
            if not delay_steps:commands=[]
        # Evaluate physical constraints at EVERY integration sample; downsample only
        # stored traces. Checking only a 50 Hz plot could miss brief slack/peaks.
        if True:
            x,v,q,w,xi,vi,force=model.unpack(y)
            xr,vr,ar,Rd,wr,al=model.reference(t)
            err=float(np.linalg.norm(x-xr));R=rotation(q)
            att=math.degrees(math.acos(float(np.clip((np.trace(Rd.T@R)-1)/2,-1,1))))
            cable,tensions,u,extensions=model.cable_forces(y)
            distances=[np.linalg.norm(xi[i]-xi[j]) for i in range(3) for j in range(i)]
            fn=np.linalg.norm(force,axis=1)
            cable_deviation=np.degrees(np.arccos(np.clip(np.sum(u*model.u0,axis=1),-1.,1.)))
            force_tilt=np.degrees(np.arccos(np.clip(force[:,2]/np.maximum(fn,1e-12),-1.,1.)))
            max_load_speed=max(max_load_speed,float(np.linalg.norm(v)))
            max_vehicle_speed=max(max_vehicle_speed,float(np.max(np.linalg.norm(vi,axis=1))))
            max_cable_deviation=max(max_cable_deviation,float(np.max(cable_deviation)))
            max_force_tilt=max(max_force_tilt,float(np.max(force_tilt)))
            max_position_error=max(max_position_error,err);error_sq+=err*err;counter+=1
            max_attitude=max(max_attitude,att);max_force=max(max_force,float(max(fn)))
            max_tension=max(max_tension,float(max(tensions)));min_tension=min(min_tension,float(min(tensions)))
            max_extension=max(max_extension,float(max(extensions)))
            min_distance=min(min_distance,float(min(distances)))
            max_residual=max(max_residual,diag['allocation_residual'])
            slack_count+=int(np.any(tensions<1e-6));sat_count+=int(np.any(diag['clipped']))
            if log_path is not None and step%log_every==0:history.append(np.r_[t,x,xr,v,tensions,fn,att,min(distances),diag['allocation_residual']])
            if not np.isfinite(y).all() or np.linalg.norm(x-xr)>5 or np.max(np.linalg.norm(vi,axis=1))>50:
                completed=False;reason='divergence_or_state_guard';break
        if step<nsteps:y=model.step(t,y,command)
    metrics=dict(completed=completed,termination=reason,simulation_end_s=t,
                 intended_duration_s=model.motion.total_time,wall_seconds=time.monotonic()-start,
                 position_rmse_m=math.sqrt(error_sq/max(counter,1)),position_peak_error_m=max_position_error,
                 attitude_peak_error_deg=max_attitude,force_peak_N=max_force,tension_min_N=min_tension,
                 load_speed_peak_m_s=max_load_speed,vehicle_speed_peak_m_s=max_vehicle_speed,
                 cable_deviation_peak_deg=max_cable_deviation,force_tilt_peak_deg=max_force_tilt,
                 tension_peak_N=max_tension,extension_peak_m=max_extension,min_vehicle_distance_m=min_distance,
                 slack_time_fraction=slack_count/max(counter,1),command_clipping_time_fraction=sat_count/max(counter,1),
                 allocation_residual_peak=max_residual,reference_distance_m=model.motion.distance,
                 reference_speed_m_s=cfg.speed,reference_acceleration_m_s2=model.motion.a_peak,
                 reference_jerk_m_s3=cfg.jerk,topology_rank=int(np.linalg.matrix_rank(model.G)),
                 model_level='rigid payload + point vehicles + elastic tension-only cables + vector-force lag',
                 config=asdict(cfg))
    if log_path is not None:
        np.savez_compressed(log_path,history=np.asarray(history),
                            columns=np.array(['time_s','load_x','load_y','load_z','ref_x','ref_y','ref_z',
                                              'load_vx','load_vy','load_vz','tension_1','tension_2','tension_3',
                                              'force_1','force_2','force_3','attitude_error_deg','min_distance','allocation_residual']))
    return metrics
