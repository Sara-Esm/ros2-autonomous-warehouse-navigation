import sys, math, re, heapq, numpy as np, cv2, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
run=sys.argv[1]
r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=run+'/bag',storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
ty={t.name:get_message(t.type) for t in r.get_all_topics_and_types()}
plans=[]; odom=[]
while r.has_next():
    t,d,ts=r.read_next()
    if t=='/plan':
        m=deserialize_message(d,ty[t]); plans.append((ts*1e-9,np.array([(p.pose.position.x,p.pose.position.y) for p in m.poses])))
    elif t=='/odom':
        m=deserialize_message(d,ty[t]); q=m.pose.pose.orientation
        odom.append((ts*1e-9,m.pose.pose.position.x,m.pose.pose.position.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z),m.twist.twist.angular.z))
odom=np.array(odom)
segs=[];cur=None
for l in open(run+'/mission.log'):
    m=re.search(r'\[(\d+\.\d+)\] \[warehouse_autonomy\]: (Navigating to|ARRIVED|MISSION HALTED)',l)
    if m:
        if m.group(2)=='Navigating to': cur=[float(m.group(1))]
        else: cur.append(float(m.group(1))); segs.append(cur)
goals=[90,90,-90,-90,90]
wrap=lambda a:(a+math.pi)%(2*math.pi)-math.pi
def turning(xy,step=0.3):
    # resample along arc length, sum |heading change|
    d=np.r_[0,np.cumsum(np.hypot(*np.diff(xy,axis=0).T))]
    s=np.arange(0,d[-1],step); x=np.interp(s,d,xy[:,0]); y=np.interp(s,d,xy[:,1])
    h=np.arctan2(np.diff(y),np.diff(x)); return h, np.sum(np.abs([wrap(a) for a in np.diff(h)]))
print('Leg | executed rotation | minimum implied by first plan (start align + path bends + final align) | ratio | DWB heading jitter')
for k,(a,b) in enumerate(segs):
    p=next(xy for w,xy in plans if w>=a and len(xy)>3)
    h,bend=turning(p)
    o=odom[(odom[:,0]>=a)&(odom[:,0]<=b)]
    start_align=abs(wrap(h[0]-o[0,3])); final=abs(wrap(math.radians(goals[k])-h[-1])) if 'ARR' else 0
    if k==4: final=0
    need=start_align+bend+final
    exe=np.sum(np.abs(wrap(np.diff(o[:,3])) if False else [wrap(v) for v in np.diff(o[:,3])]))
    print(f'  {k}: executed {math.degrees(exe):5.0f} deg | minimum {math.degrees(need):5.0f} deg (align {math.degrees(start_align):.0f} + bends {math.degrees(bend):.0f} + final {math.degrees(final):.0f}) | x{exe/need:.2f} | excess {math.degrees(exe-need):.0f} deg')
# optimal path on TRUE geometry (robot radius 0.14 + 0.05 margin), straight-line 8-connected Dijkstra
gt=np.load(run+'/../static/gt_body.npy')>0
H,W=gt.shape; res=.05; ox,oy=-7.56,-10.8
# fill outside true walls as blocked
yy,xx=np.mgrid[0:H,0:W]; X=ox+xx*res; Y=oy+(H-1-yy)*res
blk=gt|(X<-6.9)|(X>6.9)|(Y<-10.35)|(Y>10.35)
dist=cv2.distanceTransform((~blk).astype(np.uint8)*255,cv2.DIST_L2,5)*res
free=dist>0.19
def cell(x,y): return int(round(H-1-(y-oy)/res)),int(round((x-ox)/res))
def dijkstra(s,g):
    D=np.full((H,W),np.inf); D[s]=0; pq=[(0,s)]
    nb=[(-1,0,1),(1,0,1),(0,-1,1),(0,1,1),(-1,-1,1.4142),(-1,1,1.4142),(1,-1,1.4142),(1,1,1.4142)]
    while pq:
        dd,(r_,c)=heapq.heappop(pq)
        if (r_,c)==g: return dd*res
        if dd>D[r_,c]: continue
        for a_,b_,w in nb:
            rr,cc=r_+a_,c+b_
            if 0<=rr<H and 0<=cc<W and free[rr,cc] and dd+w<D[rr,cc]:
                D[rr,cc]=dd+w; heapq.heappush(pq,(dd+w,(rr,cc)))
    return float('nan')
pts=[(0,0),(-3.678,9.3),(1.803,9.3),(1.801,-9.0),(-3.63,-9.3),(-3.678,9.3)]
names=['spawn->Dock','Dock->Loading','Loading->Inventory','Inventory->Dispatch','Dispatch->Dock']
print('\nShortest collision-free path on TRUE geometry (0.19 m clearance), grid Dijkstra (overestimates ~3-5% vs any-angle):')
for n,a,b in zip(names,pts[:-1],pts[1:]):
    print(f'  {n:20s} straight {math.dist(a,b):5.2f} m, true-geometry shortest {dijkstra(cell(*a),cell(*b)):5.2f} m')
