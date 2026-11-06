from collections import deque
import importlib.util
import math
import sys
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
from test_reliability import load, TIMING
from continuous import stopping_speed, observation_speed, PassageLedger, RangeFusion, timed_rear
from watch_logic import Stats
from watch_logic import OdoHistory


class EnvelopeTests(unittest.TestCase):
    def test_speed_obeys_delayed_braking_distance(self):
        for distance in (3, 7, 11, 20):
            for delay in (.02,.1,.25):
                speed = stopping_speed(distance,delay,300)
                self.assertLessEqual(speed*delay+speed*speed/600+1,distance+1e-9)

    def test_two_frames_fit_in_visibility_window(self):
        for gap in (.08,.17,.4):
            self.assertLessEqual(observation_speed(gap)*2*gap,12.40001)  # окно камеры ±62мм

    def test_decision_does_not_commit_arrival(self):
        ledger = PassageLedger()
        ledger.schedule(1,30,{'left':True})
        self.assertEqual(ledger.advance(19),[])
        self.assertEqual(ledger.advance(29.9),[])
        self.assertEqual(ledger.advance(30),[(1,30,{'left':True})])
        self.assertEqual(ledger.advance(31),[])

    def test_late_blue_cancels_scheduled_passage(self):
        ledger = PassageLedger()
        ledger.schedule(1,30,{})
        ledger.schedule(2,60,{})
        ledger.cancel_from(1)
        self.assertEqual(ledger.advance(100),[])


class FusionTests(unittest.TestCase):
    def test_small_encoder_scale_error_does_not_accumulate_over_five_metres(self):
        fusion = RangeFusion()
        correction = 0.
        errors = []
        for i in range(100):
            true = i*5.
            estimate = true*1.03+correction
            delta,lost = fusion.update(i,10+i*.1,10+i*.1,estimate,5200-true*10,200+true*10,0,0)
            self.assertFalse(lost)
            self.assertLessEqual(abs(delta),.4)
            correction += delta
            errors.append(abs(estimate+delta-true))
        self.assertLess(max(errors),.5)

    def test_missing_walls_on_stairs_never_correct_or_trigger_drift(self):
        f=RangeFusion()
        for i in range(20):
            self.assertEqual(f.update(i,10+i*.1,10+i*.1,i*5,None,None,0,0),(0,False))

    def test_changed_scene_and_duplicate_scan_cannot_create_correction(self):
        f=RangeFusion()
        f.update(1,10,10,0,1000,200,0,0)
        self.assertEqual(f.update(1,10.1,10,20,1000,200,0,0),(0,False))
        self.assertEqual(f.update(2,10.1,10.1,5,500,250,0,0),(0,False))
        self.assertEqual(f.evidence,'scene_changed')

    def test_large_coherent_discrepancy_requests_recovery(self):
        f=RangeFusion()
        for i in range(2): f.update(i,10+i*.1,10+i*.1,i*4,1000,200,0,0)
        self.assertTrue(f.update(2,10.2,10.2,8,1000,200,0,0)[1])

    def test_rear_timestamp_does_not_mix_start_and_end_of_scan(self):
        points={170:300,174:301,178:302,182:800,188:900,190:950}
        distance,ts=timed_rear(points,10,.1)
        self.assertLess(distance,310)
        self.assertTrue(9.99<ts<10)
        self.assertEqual(timed_rear({182:300,188:300,190:300},10,.1),(None,None))


class VisionTests(unittest.TestCase):
    @unittest.skipUnless(importlib.util.find_spec('numpy'), 'нужен numpy')
    def test_empty_detection_frames_count_for_sampling_coverage(self):
        import victim_watch
        watcher=victim_watch.VictimWatcher.__new__(victim_watch.VictimWatcher)
        watcher._lock=threading.Lock()
        watcher.cams={'left':None}
        watcher.dead=False
        watcher._failed=set()
        watcher._armed={'left':True}
        watcher._armed_t={'left':10.}
        watcher._arm_id=1
        watcher._processed_t={'left':None}
        watcher._frame_gaps={'left':deque(maxlen=8)}
        watcher._pending={'left':(1,10.1,1,10.1)}
        watcher.infer=Stats()
        watcher._out=[]
        watcher._live={'left':{}}
        watcher._free={'left':threading.Event()}
        watcher._on_line('{"cam":1,"id":1,"boxes":[],"ms":80}',{1:'left'})
        watcher._pending['left']=(2,10.3,1,10.3)
        watcher._on_line('{"cam":1,"id":2,"boxes":[],"ms":80}',{1:'left'})
        with patch.object(victim_watch.time,'monotonic',return_value=10.4):
            self.assertAlmostEqual(watcher.coverage_gap({'left'}),.2)
        watcher._failed.add('left')
        self.assertEqual(watcher.coverage_gap({'left'}),math.inf)

    def module(self):
        state=SimpleNamespace(lifted=threading.Event(),watched_sides=set())
        deps={n:Mock() for n in ('driving','esp_link','maze_map','turning','victims')}
        deps.update(state=state,timing=TIMING,walls=SimpleNamespace(SIDE_THRESH=200),
            lidar_walls=SimpleNamespace(wall_present=lambda d,threshold:d is not None and d<threshold),
            victim_detect=SimpleNamespace(CONF_FAST=.5,MIN_BOX_PX=10,RING_SCORE={},SUM_TO_TYPE={}))
        return load('marks',deps),state

    def test_dead_watcher_drives_on_without_vision(self):
        """Наблюдатель умер - едем дальше без него, без остановки и чекпоинта."""
        module,state=self.module()
        trip=SimpleNamespace(watcher=SimpleNamespace(dead=True))
        self.assertIsNone(module.watch_marks_while_driving(trip))
        self.assertFalse(state.lifted.is_set())
        self.assertIsNone(trip.watcher)
        module.esp_link.brake.assert_not_called()

    def test_slow_vision_caps_speed_but_never_stops(self):
        module,state=self.module()
        watcher=Mock(dead=False)
        watcher.coverage_gap.return_value=5.0
        watcher.poll.return_value=[]
        hold=Mock()
        hold.update.return_value={'left'}
        t=SimpleNamespace(watcher=watcher,now_m=10,side_hold=hold,walls={'left':140,'right':None},
            ramp_state=None,bump_on=False,assist_mode=None,pitch_delta=0,wall_forced=False,vmark=None,
            boundary_cm=30,traveled_cm=0,watch_boundary=None,watch_armed=False)
        with patch.object(module.watch_logic,'watch_allowed',return_value=True):
            self.assertIsNone(module.watch_marks_while_driving(t))
        self.assertEqual(t.perception_cap,module.config.MOVE_MIN_CMS)
        self.assertFalse(state.lifted.is_set())

    def test_simultaneous_opposite_marks_are_queued_nearest_first(self):
        module,state=self.module()
        watcher=Mock(dead=False)
        watcher.coverage_gap.return_value=.1
        watcher.poll.return_value=[SimpleNamespace(side='left',t_frame=10),SimpleNamespace(side='right',t_frame=10)]
        module._watch_decide=Mock(side_effect=[{'side':'left','pos_mm':350},{'side':'right','pos_mm':250}])
        hold=Mock()
        hold.update.return_value={'left','right'}
        hold.frame_ok.return_value=True
        t=SimpleNamespace(watcher=watcher,now_m=10,side_hold=hold,walls={'left':140,'right':140},
            ramp_state=None,bump_on=False,assist_mode=None,pitch_delta=0,wall_forced=False,vmark=None,
            cell=(0,0),hdg=0,odo_t=Mock(),traveled_cm=20,fwd=30,boundary_cm=30,watch_boundary=None)
        module.watch_marks_while_driving(t)
        self.assertEqual(t.vmark['side'],'right')
        self.assertEqual([q['side'] for q in t.mark_queue],['left'])


class ControllerIntegrationTests(unittest.TestCase):
    def test_fusion_aligns_sector_times_and_preserves_raw_slip_evidence(self):
        module=load('drive_checks',{'state':Mock(),'esp_link':Mock()})
        history=OdoHistory();history.add(10,10);history.add(10.1,12)
        raw=OdoHistory();raw.add(10,10);raw.add(10.1,12)
        yaw=OdoHistory();yaw.add(10,10);yaw.add(10.1,10)
        fusion=Mock();fusion.update.return_value=(.3,False)
        ledger=PassageLedger();ledger.schedule(1,30,{})
        t=SimpleNamespace(front_scan_cm=10,on_slope_now=False,yaw_t=yaw,front_scan_t=10,
            v_meas=20,fwd=20,range_fusion=fusion,back_observation=(500,10.1),odo_t=history,
            walls={'front':900},scan_id=2,pitch_delta=0,traveled_cm=12,raw_odo_t=raw,
            f_anchor=None,odo_hist=deque([(10,10)]),last_progress_cm=10,
            vmark={'pos_mm':200,'stop_cm':20},mark_queue=[{'pos_mm':250}],
            cell_sides=SimpleNamespace(band_x={'left':[100]}),passages=ledger)
        module.correct_longitudinal_drift(t)
        args=fusion.update.call_args.args
        self.assertAlmostEqual(args[5],500*math.cos(math.radians(10))-20)
        self.assertEqual(raw.d,[10,12])
        self.assertAlmostEqual(t.traveled_cm,12.3)
        self.assertEqual(t.vmark,{'pos_mm':203,'stop_cm':20.3})
        self.assertEqual(t.mark_queue,[{'pos_mm':253}])
        self.assertEqual(t.passages.pending[0][1],30)

    def test_second_mark_is_serviced_before_finishing_at_tile_center(self):
        state=SimpleNamespace(scan_cnt=10,lifted=threading.Event())
        checks=Mock();checks.integrate_motion.return_value=None
        marks=Mock();marks._mark_stop.return_value=None
        lidar=Mock(); lidar.get_walls.return_value={'front':None}
        module=load('drive_stop',{'state':state,'esp_link':Mock(),'driving':Mock(),
                                  'marks':marks,'drive_checks':checks,'lidar_walls':lidar})
        t=SimpleNamespace(stopping=True,mark_first=True,rem_eff=0,ramp_state=None,
            vmark={'side':'left','pos_mm':300,'stop_cm':30},mark_queue=[{'side':'right','pos_mm':300}],
            wall_forced=False,cell=(0,0),hdg=1,target_yaw=90,traveled_cm=30,
            odometry=SimpleNamespace(last=None),t_cell=1,last_progress_t=1,assist_gap_until=1,
            assist_t0=1,assist_total_t0=1,assist_progress_t=1,odo_hist=deque(),
            side_hold=Mock(),boundary_cm=30,telem_seq=1,cells=0)
        self.assertEqual(module.check_arrived(t),'next_tick')
        self.assertEqual(t.vmark['side'],'right')
        self.assertEqual(t.cells,0)
        self.assertEqual(module.check_arrived(t),'stop')
        self.assertEqual(marks._mark_stop.call_count,2)
        self.assertEqual(t.cells,1)

    def test_plan_commits_only_after_crossing_center(self):
        module=load('drive_checks',{'state':Mock(),'esp_link':Mock()})
        ledger=PassageLedger();ledger.schedule(1,30,{'left':True})
        trip=SimpleNamespace(passages=ledger,traveled_cm=29,plan=Mock())
        module.commit_passages(trip);trip.plan.passed.assert_not_called()
        trip.traveled_cm=30.1
        module.commit_passages(trip)
        trip.plan.passed.assert_called_once_with(1,{'left':True})
