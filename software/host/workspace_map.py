import math
from kinematics import HOME_AC_DEGREES, HOME_QY_DEGREES, full_linkage_kinematics, tool_is_outwardmost

HOME_A=3.5
HOME_B=3.5
HOME_C=2.5

def joint_to_world(a,b,c,d):
    qx=HOME_AC_DEGREES+(float(a)-HOME_A)
    qy=HOME_QY_DEGREES+(float(b)-HOME_B)
    p=full_linkage_kinematics(qx,qy)
    if not tool_is_outwardmost(p):
        raise ValueError("tool not outwardmost")
    r=float(p.tool.y); z=float(p.tool.z)
    yaw=math.radians((float(c)-HOME_C))
    return (r*math.sin(yaw), float(d)+r*math.cos(yaw), z)

def sample_workspace(boundary_poly, yield_mm=5.0, a_step=3.0, b_step=3.0,
                     c_step=15.0, d_step=25.0, c_min=2.5, c_max=272.0,
                     d_min=0.0, d_max=235.0, max_ui_points=12000):
    def seg_dist(px,py,ax,ay,bx,by):
        vx,vy=bx-ax,by-ay; wx,wy=px-ax,py-ay
        vv=vx*vx+vy*vy
        if vv<=1e-12:return math.hypot(px-ax,py-ay)
        t=max(0.0,min(1.0,(wx*vx+wy*vy)/vv))
        return math.hypot(px-(ax+t*vx),py-(ay+t*vy))
    def inside(x,y):
        hit=False; j=len(boundary_poly)-1
        for i in range(len(boundary_poly)):
            xi,yi=boundary_poly[i]; xj,yj=boundary_poly[j]
            if ((yi>y)!=(yj>y)) and x < (xj-xi)*(y-yi)/(yj-yi)+xi:
                hit=not hit
            j=i
        return hit
    def score(x,y):
        d=min(seg_dist(x,y,*boundary_poly[i],*boundary_poly[(i+1)%len(boundary_poly)])
              for i in range(len(boundary_poly)))
        return d if inside(x,y) else -d

    amin=min(p[0] for p in boundary_poly); amax=max(p[0] for p in boundary_poly)
    bmin=min(p[1] for p in boundary_poly); bmax=max(p[1] for p in boundary_poly)
    # Each safe sample keeps the arm pose that generated the side-plane tool point:
    # radius, Z, A, B.  This makes world click planning cheap.
    arm=[]; a=amin
    while a<=amax+1e-9:
        b=bmin
        while b<=bmax+1e-9:
            if score(a,b)>=yield_mm:
                try:
                    p=full_linkage_kinematics(HOME_AC_DEGREES+(a-HOME_A),
                                              HOME_QY_DEGREES+(b-HOME_B))
                    if tool_is_outwardmost(p):
                        arm.append((float(p.tool.y),float(p.tool.z),float(a),float(b)))
                except Exception:
                    pass
            b+=b_step
        a+=a_step

    cs=[]; c=c_min
    while c<=c_max+1e-9:
        cs.append(c); c+=c_step
    if not cs or cs[-1]<c_max: cs.append(c_max)
    ds=[]; d=d_min
    while d<=d_max+1e-9:
        ds.append(d); d+=d_step
    if not ds or ds[-1]<d_max: ds.append(d_max)

    total=len(arm)*len(cs)*len(ds)
    stride=max(1,total//max_ui_points)
    points=[]; xmin=ymin=zmin=float("inf"); xmax=ymax=zmax=float("-inf"); k=0
    for c in cs:
        th=math.radians((c-HOME_C)); sn=math.sin(th); co=math.cos(th)
        for d in ds:
            for r,z,a,b in arm:
                x=r*sn; y=d+r*co
                xmin=min(xmin,x);xmax=max(xmax,x)
                ymin=min(ymin,y);ymax=max(ymax,y)
                zmin=min(zmin,z);zmax=max(zmax,z)
                if k%stride==0 and len(points)<max_ui_points:
                    points.append((round(x,2),round(y,2),round(z,2),round(c,2),round(d,2)))
                k+=1

    return {
        "meta":{"ab_samples":len(arm),"c_samples":len(cs),"d_samples":len(ds),
                "total_samples":total,"ui_samples":len(points),"yield":yield_mm,
                "c_range":[c_min,c_max],"d_range":[d_min,d_max],
                "planner_samples":len(arm)},
        "bounds":{"x":[xmin,xmax],"y":[ymin,ymax],"z":[zmin,zmax]},
        "side_samples":[(round(r,3),round(z,3),round(a,3),round(b,3))
                        for r,z,a,b in arm],
        "points":points
    }
