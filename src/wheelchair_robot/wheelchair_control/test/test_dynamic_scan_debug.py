"""Debug instrumentation preserves tracking/classification and exposes actual inputs."""
import json
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
from sensor_msgs.msg import LaserScan
from wheelchair_control.dynamic_scan_filter import DynamicScanFilter


def test_debug_records_observation_displacement_and_tf_without_reclassifying():
    # Exercise the real tracking and serialization methods without starting DDS.
    n=SimpleNamespace(tracks=[],q_accel=.6,r_std=.1,gate=.5,track_timeout=.4,
        publish_debug_enabled=True,debug_sequence=0,debug_odom_stamp_ns=900_000_000,
        debug_odom_child_frame='base_link',base_frame='base_footprint',odom_frame='odom',
        robot_v_base=np.array([.15,0.]),robot_p_odom=np.array([1.,0.]),have_odom=True,
        debug_tf={'laser_lookup':'latest_laser'},max_range=1.5,v_dynamic_thresh=.3,
        v_static=.1,tcpa_max=4.,dcpa_max=.3,dynamic_hold_sec=5.,min_age=3,
        seg_gap=.15,min_seg_pts=2,max_width=.8,merge_dist=.6,max_merged_width=1.3,
        pub_debug=Mock(),get_clock=lambda: SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1100000000)),
        get_logger=lambda:Mock())
    DynamicScanFilter.track_step(n,[{'z':np.array([.8,.1]),'beams':[0,1]}],1.)
    DynamicScanFilter.track_step(n,[{'z':np.array([.85,.1]),'beams':[0,1]}],1.1)
    DynamicScanFilter.evaluate_threat(n,0.,1.1)
    before=n.tracks[0].X.copy(); classification=n.tracks[0].reason
    scan=LaserScan(); scan.header.frame_id='laser'; scan.header.stamp.sec=1
    DynamicScanFilter.publish_debug(n,scan,123,[],'ok')
    d=json.loads(n.pub_debug.publish.call_args.args[0].data)
    assert d['tracks'][0]['displacement_odom'][0]>.04
    assert d['tracks'][0]['confidence'] is None
    assert d['tracks'][0]['reason_code']==classification
    assert d['tf_used']['laser_lookup']=='latest_laser'
    assert d['robot_velocity_frame']=='base_link'
    assert np.array_equal(before,n.tracks[0].X)
    DynamicScanFilter.publish_debug(n,scan,124,[],'tf-unavailable-passthrough')
    assert json.loads(n.pub_debug.publish.call_args.args[0].data)['tracks']==[]
