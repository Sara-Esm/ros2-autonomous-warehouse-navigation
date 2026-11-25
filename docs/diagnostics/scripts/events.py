import sys, math, re, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
run=sys.argv[1]
ms=float(re.search(r'mission_start_wall: ([\d.]+)', open(run+'/meta.txt').read()).group(1))
r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=run+'/bag',storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
ty={t.name:get_message(t.type) for t in r.get_all_topics_and_types()}
mo=[]; odom=[]
print('BT events (wall s since mission start):')
last=None
while r.has_next():
    t,d,ts=r.read_next(); w=ts*1e-9
    if t=='/behavior_tree_log':
        for e in deserialize_message(d,ty[t]).event_log:
            if e.node_name in ('Spin','Wait','BackUp','ClearLocalCostmap-Context','ClearGlobalCostmap-Context','NavigateWithReplanning','FollowPath','ComputePathToPose') and e.current_status in ('FAILURE','RUNNING') :
                if e.node_name=='ComputePathToPose' and e.current_status=='RUNNING': continue
                if e.node_name=='FollowPath' and e.current_status=='RUNNING': continue
                print(f'  {w-ms:7.1f}  {e.node_name:28s} {e.previous_status}->{e.current_status}')
    elif t=='/tf':
        for tr in deserialize_message(d,ty[t]).transforms:
            if tr.header.frame_id=='map':
                q=tr.transform.rotation; mo.append((w,tr.transform.translation.x,tr.transform.translation.y,math.degrees(math.atan2(2*q.w*q.z,1-2*q.z*q.z))))
            elif tr.header.frame_id=='odom':
                odom.append((w,tr.transform.translation.x,tr.transform.translation.y))
mo=np.array(mo); od=np.array(odom)
j=np.hypot(np.diff(mo[:,1]),np.diff(mo[:,2])); yj=np.abs(np.diff(mo[:,3]))
print('map->odom jumps > 15 cm or > 2 deg:')
for i in np.nonzero((j>0.15)|(yj>2))[0]:
    k=np.searchsorted(od[:,0],mo[i+1,0]); print(f'  {mo[i+1,0]-ms:7.1f}s  dxy {j[i]*100:5.1f} cm dyaw {mo[i+1,3]-mo[i,3]:+5.2f} deg  robot true ({od[min(k,len(od)-1),1]:+.2f},{od[min(k,len(od)-1),2]:+.2f}); map->odom now ({mo[i+1,1]:+.2f},{mo[i+1,2]:+.2f},{mo[i+1,3]:+.1f})')
