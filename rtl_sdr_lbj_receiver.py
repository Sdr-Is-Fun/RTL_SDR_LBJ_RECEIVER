#!/usr/bin/env python3
# SDR-LBJ
# Copyright (C) 2026 <你的名字或GitHub ID>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

from __future__ import annotations
import argparse, os, sys, time, socket, struct, threading, re, unicodedata, queue
import subprocess
import numpy as np
from numpy.typing import NDArray
import scipy.signal as ssig
from scipy.signal import firwin, lfilter, lfilter_zi
if os.name == 'nt':
    os.system('')
BASEBAND_RATE = 48000
BAUD_RATE = 1200
HW_GAIN_DB = 15.7
PPM = 1
TCP_HOST = '127.0.0.1'
TCP_PORT = 1234
RTL_SAMPLE_RATE = 960000
HALFBAND_STAGES = 2
MID_RATE = 240000
BLOCK_SIZE = 65536
DEFAULT_DC_OFFSET_HZ = 50000.0
DEFAULT_BW_KHZ = 19.5
DEFAULT_AFC_MAX_HZ = 1500.0
DEFAULT_RSSI_THRESHOLD_DB = -55.0
DEFAULT_RSSI_HYST_DB = 4.0
DEFAULT_RSSI_HOLD_MS = 700.0
DEFAULT_ETA_MAX_SECONDS = 6 * 3600
R820T_GAINS = [0.0, 0.9, 1.4, 2.7, 3.7, 7.7, 8.7, 12.5, 14.4, 15.7, 16.6, 19.7, 20.7, 22.9, 25.4, 28.0, 29.7, 32.8, 33.8, 36.4, 37.2, 38.6, 40.2, 42.1, 43.4, 43.9, 44.5, 48.0, 49.6]
CMD_SET_FREQ = 1
CMD_SET_SAMPLERATE = 2
CMD_SET_GAINMODE = 3
CMD_SET_GAIN = 4
CMD_SET_FREQCORR = 5
CMD_SET_AGC = 8
_g0 = {'running': True, 'keywords': [], 'input_mode': False, 'filter_mode': 'highlight', 'strict_filter': True, 'show_err_warn': True, 'show_help': False}
_g1 = {'train': '----', 'direction': '未知', 'speed': '---', 'position': '---.-', 'loco': '----', 'loco_code': '---', 'route': '----', 'route_valid': False, 'is_detailed': False}
_g2 = {'freq': 821237500.0, 'gain': HW_GAIN_DB, 'ppm': PPM, 'sample_rate_k': 960, 'rssi': -140.0, 'cs_threshold': DEFAULT_RSSI_THRESHOLD_DB, 'rssi_gate': 'OFF', 'rssi_hold_ms': 0.0, 'afc_hz': 0.0, 'afc_err_hz': 0.0, 'afc_score': 0.0, 'user_km': None, 'current_route': '----', 'current_route_km': None, 'current_route_km_text': '---', 'route_km_map': {}, 'known_routes': [], 'eta_seconds': None, 'eta_time': '--:--:--', 'eta_distance_km': None, 'eta_status': '未设置线路位置', 'eta_train': '----', 'eta_route': '----', 'train': '----', 'direction': '未知', 'speed': '---', 'position': '---.-', 'loco': '----', 'loco_code': '---', 'route': '----', 'category': '等待信号...', 'is_detailed': False, 'is_hit': False, 'warning': '', 'warning_time': 0.0}
_g3 = 32
_g4 = {'smoothed': np.full(_g3, -120.0, dtype=np.float64)}

def _u0(t):
    return sum((2 if unicodedata.east_asian_width(c) in 'WF' else 1 for c in str(t)))

def _u1(t, w):
    return str(t) + ' ' * max(0, w - _u0(str(t)))

def _u2(t, w):
    s = str(t)
    out = []
    used = 0
    for c in s:
        cw = 2 if unicodedata.east_asian_width(c) in 'WF' else 1
        if used + cw > w:
            break
        out.append(c)
        used += cw
    return ''.join(out)

def _u3(t, w):
    s = _u2(t, w)
    return s + ' ' * max(0, w - _u0(s))

def _u4():
    if os.name == 'nt':
        try:
            import msvcrt
            if not msvcrt.kbhit():
                return None
            ch = msvcrt.getwch()
            if ch in ('\x00', 'à'):
                if msvcrt.kbhit():
                    msvcrt.getwch()
                return None
            return ch.lower()
        except Exception:
            return None
    try:
        import select, tty, termios
        fd = sys.stdin.fileno()
        old_attr = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            if select.select([sys.stdin], [], [], 0.0)[0]:
                ch = sys.stdin.read(1)
                return ch.lower() if ch else None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)
    except Exception:
        pass
    return None

def _u5(train_no, is_detailed):
    if not train_no or train_no in ('等待信号...', '----'):
        return '等待信号...'
    tn = train_no.replace(' ', '').strip().upper()
    for p, c in [('K', '快速旅客列车'), ('Z', '直达特快列车'), ('T', '特快旅客列车'), ('G', '高速动车组'), ('D', '动车组列车'), ('C', '城际动车组'), ('X', '行包货物列车'), ('Y', '旅游旅客列车')]:
        if tn.startswith(p):
            return c
    if tn.startswith('00'):
        d = ''.join(filter(str.isdigit, tn))
        if d:
            n = int(d)
            if 1 <= n <= 100:
                return '动车组有火回送'
            if 101 <= n <= 198:
                return '动车组无火跨局回送'
            if 201 <= n <= 298:
                return '动车组无火管内回送'
            if 301 <= n <= 398:
                return '跨局回送客车'
            if 401 <= n <= 498:
                return '管内回送客车'
    d = ''.join(filter(str.isdigit, tn))
    if d:
        n = int(d)
        ranges = [(10001, 19998, '技术直达货运列车'), (20001, 29998, '直通货运列车'), (30001, 39998, '区段货运列车'), (40001, 44998, '摘挂列车'), (45001, 49998, '小运转列车'), (50001, 50998, '客车单机'), (51001, 51998, '货车单机'), (52001, 52998, '小运转单机'), (53001, 54998, '补机'), (55001, 55998, '试运行列车'), (56001, 56998, '轨道车、小型工程车'), (57001, 57998, '路用列车'), (58101, 58998, '救援列车'), (60001, 69998, '工厂自备车'), (70001, 70998, '超限货运列车'), (71001, 72998, '万吨货物列车'), (73001, 74998, '冷藏列车'), (75001, 75998, '集装箱专列'), (80001, 81748, '直达货运/五定班列'), (81751, 81998, '快运货运列车'), (82001, 84998, '煤炭直达列车'), (85001, 85998, '石油直达列车'), (86001, 86998, '始发直达列车'), (87001, 87998, '空车直达列车'), (88001, 88998, '汽运专列'), (90001, 91998, '军用列车(满载)'), (93001, 94998, '军用列车(空车)')]
        for lo, hi, cat in ranges:
            if lo <= n <= hi:
                return cat
        if 1 <= n <= 9998 and (not tn.startswith('00')):
            return '普通旅客列车' if is_detailed else '等待详细报文确认...'
    return '未知类别'

class _A0:

    def __init__(self, user_km=None, max_seconds=DEFAULT_ETA_MAX_SECONDS, route_km_map=None):
        self.default_km = None if user_km is None else float(user_km)
        self.down_increases = True
        self.max_seconds = float(max_seconds)
        self.route_km_map = {}
        self.known_routes = []
        if route_km_map:
            for route, km in route_km_map.items():
                self._a9(route, km)
        self._a7('----')

    @staticmethod
    def _a0(route):
        if route is None:
            return ''
        r = str(route).replace('\x00', '').strip()
        if not r or r in ('----', '---', '未知', '等待信号...'):
            return ''
        return r

    @staticmethod
    def _a1(km):
        try:
            return f'{float(km):06.1f}KM'
        except Exception:
            return '---'

    @staticmethod
    def _a2(v):
        if v is None:
            return None
        s = str(v).upper().replace('KM', '').strip()
        if not s or s in ('---', '---.-', '----'):
            return None
        try:
            km = float(s)
        except Exception:
            return None
        if km < 0 or km > 9999.9:
            return None
        return round(km, 1)

    @staticmethod
    def _a3(items):
        result = {}
        if not items:
            return result
        if isinstance(items, str):
            items = [items]
        for item in items:
            for part in str(item).split(','):
                part = part.strip()
                if not part or '=' not in part:
                    continue
                route, val = part.split('=', 1)
                route = _A0._a0(route)
                km = _A0._a2(val)
                if route and km is not None:
                    result[route] = km
        return result

    @staticmethod
    def _a4(route):
        route = _A0._a0(route)
        if not route:
            return False
        if len(route) > 24:
            return False
        if re.search('[\\*U\\(\\)<>\\[\\]{}\\\\/|;:,.，。！？?！@#$%^&_=+`~]', route):
            return False
        if not re.search('[\\u4e00-\\u9fa5A-Za-z0-9]', route):
            return False
        return bool(re.match('^[\\u4e00-\\u9fa5A-Za-z0-9\\-\\s]+$', route))

    def _a5(self, route):
        route = self._a0(route)
        if not self._a4(route):
            return ''
        if route and route not in self.known_routes:
            self.known_routes.append(route)
            self.known_routes = self.known_routes[-20:]
        _g2['known_routes'] = list(self.known_routes)
        return route

    def _a6(self, route):
        route = self._a0(route)
        if route and route in self.route_km_map:
            return self.route_km_map[route]
        return self.default_km

    def _a7(self, route):
        route = self._a0(route) or '----'
        km = self._a6(route)
        _g2['current_route'] = route
        _g2['user_km'] = km
        _g2['current_route_km'] = km
        _g2['current_route_km_text'] = self._a1(km) if km is not None else '---'
        _g2['route_km_map'] = dict(self.route_km_map)
        _g2['known_routes'] = list(self.known_routes)
        if km is None:
            _g2.update({'eta_seconds': None, 'eta_time': '--:--:--', 'eta_distance_km': None, 'eta_status': '未设置线路位置' if route != '----' else '线路未知', 'eta_train': '----', 'eta_route': route})

    def _a8(self, user_km):
        km = self._a2(user_km)
        self.default_km = km
        self._a7(_g2.get('route'))

    def _a9(self, route, km):
        route = self._a0(route)
        km = self._a2(km)
        if not route or km is None:
            return False
        self.route_km_map[route] = km
        self._a5(route)
        self._a7(_g2.get('route'))
        return True

    def _aa(self, route):
        route = self._a0(route)
        if route and route in self.route_km_map:
            del self.route_km_map[route]
        self._a7(_g2.get('route'))

    @staticmethod
    def _ab(v):
        if v is None:
            return None
        s = str(v).replace('KM', '').replace('km', '').strip()
        if not s or s in ('---', '---.-', '----'):
            return None
        try:
            return float(s)
        except Exception:
            return None

    @staticmethod
    def _ac(ts):
        return time.strftime('%H:%M:%S', time.localtime(ts))

    def _ad(self, good_data=True, now=None):
        route = self._a0(_g2.get('route'))
        route_good = bool(good_data and self._a4(route))
        if route_good:
            self._a5(route)
        self._a7(route if route_good or route in self.route_km_map else route)
        if not good_data:
            _g2['eta_status'] = '错包忽略'
            return False
        if not route:
            _g2['eta_status'] = '线路未知'
            return False
        user_km = self._a6(route)
        if user_km is None:
            _g2.update({'eta_seconds': None, 'eta_time': '--:--:--', 'eta_distance_km': None, 'eta_status': '未设置线路位置', 'eta_route': route})
            return False
        train = _g2.get('train', '----')
        direction = _g2.get('direction', '未知')
        speed = self._ab(_g2.get('speed'))
        train_km = self._ab(_g2.get('position'))
        if not train or train == '----':
            _g2['eta_status'] = '等待车次'
            return False
        if direction not in ('上行', '下行'):
            _g2['eta_status'] = '方向未知'
            return False
        if speed is None or speed <= 0:
            _g2['eta_status'] = '速度无效'
            return False
        if train_km is None:
            _g2['eta_status'] = '公里标无效'
            return False
        if self.down_increases:
            distance_km = user_km - train_km if direction == '下行' else train_km - user_km
        else:
            distance_km = train_km - user_km if direction == '下行' else user_km - train_km
        if distance_km < -0.02:
            _g2.update({'eta_seconds': None, 'eta_time': '--:--:--', 'eta_distance_km': None, 'eta_status': '远离/已过', 'eta_train': train, 'eta_route': route})
            return False
        distance_km = max(0.0, distance_km)
        eta_seconds = distance_km / speed * 3600.0
        if eta_seconds > self.max_seconds:
            _g2.update({'eta_seconds': None, 'eta_time': '--:--:--', 'eta_distance_km': distance_km, 'eta_status': 'ETA过大', 'eta_train': train, 'eta_route': route})
            return False
        if now is None:
            now = time.time()
        eta_time = self._ac(now + eta_seconds)
        _g2.update({'eta_seconds': eta_seconds, 'eta_time': eta_time, 'eta_distance_km': distance_km, 'eta_status': '即将到达' if eta_seconds <= 10 else '接近', 'eta_train': train, 'eta_route': route})
        return True

class _D0:

    def __init__(self):
        self._dc_i = self._dc_q = 0.0
        self._gr = 1.0

    def process(self, iq):
        i, q = (iq.real.astype(np.float64), iq.imag.astype(np.float64))
        self._dc_i += 0.001 * (np.mean(i) - self._dc_i)
        self._dc_q += 0.001 * (np.mean(q) - self._dc_q)
        i -= self._dc_i
        q -= self._dc_q
        pi, pq = (np.mean(i * i), np.mean(q * q))
        if pi > 1e-20:
            self._gr += 0.0001 * (np.sqrt(pq / pi) - self._gr)
            self._gr = max(0.9, min(1.1, self._gr))
            q *= self._gr
        return (i + 1j * q).astype(np.complex64)

class _D1:

    def __init__(self, sr, offset=0.0):
        self.sr = float(sr)
        self.offset = float(offset)
        self._ph = 0.0
        self._update_inc()

    def _update_inc(self):
        self._inc = -2.0 * np.pi * self.offset / self.sr

    def set_offset(self, offset_hz):
        self.offset = float(offset_hz)
        self._update_inc()

    def add_offset(self, delta_hz):
        self.set_offset(self.offset + float(delta_hz))

    def process(self, iq):
        if abs(self.offset) < 0.1:
            return iq
        n = len(iq)
        ph = self._ph + self._inc * np.arange(n, dtype=np.float64)
        self._ph = (ph[-1] + self._inc) % (2.0 * np.pi)
        lo = np.exp(1j * ph).astype(np.complex64)
        return iq * lo

class _D2:

    def __init__(self, sr, bw=15000.0):
        cut = min(bw / 2.0, sr * 0.45)
        tw = max(bw * 0.1, 1000)
        nt = max(31, min(int(4 * sr / tw), 511)) | 1
        self.h = firwin(nt, cut, fs=sr, window='blackmanharris').astype(np.float32)
        self._z = np.zeros(nt - 1, dtype=np.complex64)

    def process(self, iq):
        out, self._z = lfilter(self.h, 1.0, iq, zi=self._z)
        return out

class _D3:

    def __init__(self, nt=63):
        self.h = firwin(nt, 0.25, window='blackmanharris').astype(np.float32)
        self._z = np.zeros(nt - 1, dtype=np.complex64)

    def process(self, iq):
        out, self._z = lfilter(self.h, 1.0, iq, zi=self._z)
        return out[::2]

class _D4:

    def __init__(self, sr_in, sr_out):
        self.d = sr_in // sr_out
        cut = sr_out / 2 * 0.9
        tw = sr_out * 0.1
        nt = max(31, min(int(4 * sr_in / tw), 255)) | 1
        self.h = firwin(nt, cut, fs=sr_in, window='blackmanharris').astype(np.float32)
        self._z = np.zeros(nt - 1, dtype=np.complex64)

    def process(self, iq):
        out, self._z = lfilter(self.h, 1.0, iq, zi=self._z)
        return out[::self.d]

class _D5:

    def __init__(self):
        self._prev = np.complex64(0)

    def process(self, iq):
        buf = np.empty(len(iq) + 1, dtype=np.complex64)
        buf[0] = self._prev
        buf[1:] = iq
        self._prev = iq[-1]
        return (np.angle(buf[1:] * np.conj(buf[:-1])) * (1.0 / np.pi)).astype(np.float32)

class _A1:

    def __init__(self, on_db=DEFAULT_RSSI_THRESHOLD_DB, hysteresis_db=DEFAULT_RSSI_HYST_DB, hold_ms=DEFAULT_RSSI_HOLD_MS, confirm_blocks=1, enabled=True):
        self.on_db = float(on_db)
        self.hysteresis_db = float(hysteresis_db)
        self.off_db = self.on_db - self.hysteresis_db
        self.hold_ms = float(hold_ms)
        self.confirm_blocks = max(1, int(confirm_blocks))
        self.enabled = bool(enabled)
        self.active = False
        self.hold_left_ms = 0.0
        self._on_count = 0
        self.just_activated = False
        self.just_deactivated = False
        self.state = 'BYPASS' if not self.enabled else 'OFF'

    def _ae(self, on_db):
        self.on_db = float(on_db)
        self.off_db = self.on_db - self.hysteresis_db

    def reset(self):
        self.active = False
        self.hold_left_ms = 0.0
        self._on_count = 0
        self.just_activated = False
        self.just_deactivated = False
        self.state = 'BYPASS' if not self.enabled else 'OFF'

    def update(self, rssi_db, audio_len, fs_audio=BASEBAND_RATE):
        self.just_activated = False
        self.just_deactivated = False
        if not self.enabled:
            self.active = True
            self.hold_left_ms = self.hold_ms
            self.state = 'BYPASS'
            return True
        block_ms = 1000.0 * max(0, int(audio_len)) / float(fs_audio)
        was_active = self.active
        if not self.active:
            if rssi_db >= self.on_db:
                self._on_count += 1
                if self._on_count >= self.confirm_blocks:
                    self.active = True
                    self.hold_left_ms = self.hold_ms
            else:
                self._on_count = 0
        elif rssi_db >= self.off_db:
            self.hold_left_ms = self.hold_ms
            self._on_count = self.confirm_blocks
        else:
            self.hold_left_ms -= block_ms
            if self.hold_left_ms <= 0.0:
                self.active = False
                self.hold_left_ms = 0.0
                self._on_count = 0
        self.just_activated = not was_active and self.active
        self.just_deactivated = was_active and (not self.active)
        if self.active:
            self.state = 'ON' if rssi_db >= self.off_db else 'HOLD'
        else:
            self.state = 'ARM' if self._on_count > 0 else 'OFF'
        return self.active

class _D6:

    def __init__(self, fs, base_ddc_hz, max_afc_hz=1500.0, loop_gain=0.45, max_step_hz=350.0, min_alt_score=0.25):
        self.fs = float(fs)
        self.base_ddc_hz = float(base_ddc_hz)
        self.max_afc_hz = float(max_afc_hz)
        self.loop_gain = float(loop_gain)
        self.max_step_hz = float(max_step_hz)
        self.min_alt_score = float(min_alt_score)
        self.afc_hz = 0.0
        self.dc_norm = 0.0
        self.last_err_hz = 0.0
        self.last_score = 0.0
        self.updated = False
        self.enabled = True

    @staticmethod
    def _trimmed_mean(x, trim=0.1):
        if len(x) == 0:
            return 0.0
        xs = np.sort(np.asarray(x, dtype=np.float32))
        k = int(len(xs) * trim)
        if k > 0 and len(xs) > 2 * k:
            xs = xs[k:-k]
        return float(np.mean(xs)) if len(xs) else 0.0

    def process(self, y, ddc: _D1, update_allowed=True):
        self.updated = False
        yy = np.asarray(y, dtype=np.float32)
        if not self.enabled:
            return yy.astype(np.float32, copy=False)
        if len(yy) < 160:
            return (yy - self.dc_norm).astype(np.float32)
        if not update_allowed:
            self.last_score = 0.0
            self.last_err_hz = self.dc_norm * self.fs * 0.5
            return (yy - self.dc_norm).astype(np.float32)
        finite = yy[np.isfinite(yy)]
        finite = finite[np.abs(finite) < 0.95]
        if len(finite) < 160:
            return (yy - self.dc_norm).astype(np.float32)
        dc_now = self._trimmed_mean(finite, trim=0.1)
        max_norm = 2.0 * self.max_afc_hz / self.fs
        dc_now = float(np.clip(dc_now, -max_norm, max_norm))
        yc = yy - dc_now
        spb = max(1, int(round(self.fs / BAUD_RATE)))
        if len(yc) > spb:
            pwr = float(np.mean(yc * yc)) + 1e-12
            score = -float(np.mean(yc[:-spb] * yc[spb:])) / pwr
        else:
            score = 0.0
        self.last_score = score
        alpha_dc = 1.0 if score >= self.min_alt_score else 0.01
        self.dc_norm += alpha_dc * (dc_now - self.dc_norm)
        self.dc_norm = float(np.clip(self.dc_norm, -max_norm, max_norm))
        if score >= self.min_alt_score:
            f_err_hz = dc_now * self.fs * 0.5
            f_err_hz = float(np.clip(f_err_hz, -self.max_afc_hz, self.max_afc_hz))
            self.last_err_hz = f_err_hz
            step = self.loop_gain * f_err_hz
            step = float(np.clip(step, -self.max_step_hz, self.max_step_hz))
            old_afc = self.afc_hz
            self.afc_hz = float(np.clip(self.afc_hz + step, -self.max_afc_hz, self.max_afc_hz))
            if abs(self.afc_hz - old_afc) >= 5.0:
                ddc.set_offset(self.base_ddc_hz + self.afc_hz)
                self.updated = True
        else:
            self.last_err_hz = self.dc_norm * self.fs * 0.5
        return (yy - self.dc_norm).astype(np.float32)

class _D7:

    def __init__(self, sample_rate=RTL_SAMPLE_RATE, halfband_n=HALFBAND_STAGES, mid_rate=MID_RATE, dc_offset=DEFAULT_DC_OFFSET_HZ, user_offset=0.0, bw=DEFAULT_BW_KHZ * 1000.0, rssi_offset=0.0, afc_enable=True, afc_max_hz=DEFAULT_AFC_MAX_HZ, afc_gain=0.45):
        self.rssi_offset = rssi_offset
        self.base_ddc = float(dc_offset + user_offset)
        self.iq_corr = _D0()
        self.ddc = _D1(sample_rate, self.base_ddc)
        self.hbs = [_D3(63) for _ in range(halfband_n)]
        self.ch_flt = _D2(mid_rate, bw)
        self.fir_dec = _D4(mid_rate, BASEBAND_RATE)
        self.fm = _D5()
        self.afc = _D6(BASEBAND_RATE, self.base_ddc, max_afc_hz=afc_max_hz, loop_gain=afc_gain)
        self.afc.enabled = bool(afc_enable)
        ch = f'{sample_rate / 1000.0:.0f}k→DDC({self.base_ddc / 1000.0:+.0f}k+AFC)'
        if halfband_n > 0:
            ch += f'→HB×{halfband_n}→{mid_rate / 1000.0:.0f}k'
        ch += f'→CH({bw / 1000.0:.1f}k)→FIR÷{mid_rate // BASEBAND_RATE}→{BASEBAND_RATE / 1000.0:.0f}k→FM→AFC/DC'
        self.chain_desc = ch

    def process(self, iq, rssi_gate=None):
        x = self.iq_corr.process(iq)
        x = self.ddc.process(x)
        for h in self.hbs:
            x = h.process(x)
        x = self.ch_flt.process(x)
        x = self.fir_dec.process(x)
        rssi = float(10.0 * np.log10(np.vdot(x, x).real / max(1, len(x)) + 1e-12) + self.rssi_offset)
        y = self.fm.process(x)
        rx_active = True
        if rssi_gate is not None:
            rx_active = rssi_gate.update(rssi, len(y), BASEBAND_RATE)
        y = self.afc.process(y, self.ddc, update_allowed=rx_active)
        return (y, rssi, rx_active)

    def reset_afc(self):
        self.afc.afc_hz = 0.0
        self.afc.dc_norm = 0.0
        self.afc.last_err_hz = 0.0
        self.afc.last_score = 0.0
        self.afc.updated = False
        self.ddc.set_offset(self.base_ddc)

    def consume_afc_updated(self):
        updated = bool(self.afc.updated)
        self.afc.updated = False
        return updated

class _A2:

    def __init__(self, host, port, freq_hz, sample_rate, block_size, dc_offset=0.0):
        self.host = host
        self.port = port
        self.freq_hz = freq_hz
        self.dc_offset = dc_offset
        self.sample_rate = sample_rate
        self.block_size = block_size
        self._q = queue.Queue(maxsize=15)
        self._running = False
        self._thread = None
        self._sock = None
        self._error = None

    def _af(self, target_freq):
        hw_freq = target_freq - self.dc_offset
        self.freq_hz = hw_freq
        self._send_cmd(CMD_SET_FREQ, int(hw_freq))

    def _ag(self, g):
        nearest = min(R820T_GAINS, key=lambda x: abs(x - g))
        self._send_cmd(CMD_SET_GAINMODE, 1)
        self._send_cmd(CMD_SET_GAIN, int(nearest * 10))
        return nearest

    def _ah(self, p):
        self._send_cmd(CMD_SET_FREQCORR, int(p))

    def _start_android_driver(self):
        uri = f'iqsrc://-a 127.0.0.1 -p {self.port} -s {int(self.sample_rate)} -f {int(self.freq_hz)} -T 0'
        try:
            subprocess.run(['am', 'start', '-a', 'android.intent.action.VIEW', '-d', uri], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2)
            time.sleep(3.0)
        except:
            pass

    def _reader_task(self):
        chunk_bytes = self.block_size * 2
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(5.0)
            self._sock.connect((self.host, self.port))
            hdr = b''
            while len(hdr) < 12:
                buf = self._sock.recv(12 - len(hdr))
                if not buf:
                    self._error = 'TCP连接断开'
                    return
                hdr += buf
            self._sock.settimeout(None)
            self._send_cmd(CMD_SET_SAMPLERATE, self.sample_rate)
            self._send_cmd(CMD_SET_FREQ, int(self.freq_hz))
            self._send_cmd(CMD_SET_GAINMODE, 1)
            self._send_cmd(CMD_SET_GAIN, int(_g2.get('gain', HW_GAIN_DB) * 10))
            self._send_cmd(CMD_SET_FREQCORR, int(_g2.get('ppm', PPM)))
            self._send_cmd(CMD_SET_AGC, 0)
            stream = self._sock.makefile('rb')
            while self._running:
                data = stream.read(chunk_bytes)
                if not data:
                    if self._running:
                        self._error = 'rtl_tcp 服务端已断开'
                        break
                if len(data) < chunk_bytes:
                    continue
                iq = np.frombuffer(data, dtype=np.uint8).astype(np.float32)
                iq_c = (iq[0::2] - 127.5) / 128.0 + 1j * (iq[1::2] - 127.5) / 128.0
                try:
                    self._q.put(iq_c.astype(np.complex64), timeout=1.0)
                except queue.Full:
                    pass
        except ConnectionRefusedError:
            self._error = '【连接被拒】请确认是否已授权 USB'
        except Exception as e:
            if self._running:
                self._error = f'网络异常: {e}'

    def _send_cmd(self, cmd_id, param):
        if self._sock:
            try:
                self._sock.sendall(struct.pack('>BI', cmd_id, param & 4294967295))
            except:
                pass

    def open(self):
        self._running = True
        self._start_android_driver()
        self._thread = threading.Thread(target=self._reader_task, daemon=True)
        self._thread.start()

    def read(self):
        for _ in range(20):
            if self._error:
                raise RuntimeError(self._error)
            try:
                return self._q.get(timeout=0.5)
            except queue.Empty:
                continue
        raise RuntimeError('等待数据流超时，硬件可能未授权。')

    def close(self):
        self._running = False
        if self._sock:
            try:
                self._sock.close()
            except:
                pass
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)

def _popcount32(x):
    x = x & 4294967295
    x = x - (x >> 1 & 1431655765)
    x = (x & 858993459) + (x >> 2 & 858993459)
    x = x + (x >> 4) & 252645135
    return x * 16843009 >> 24 & 255

def _calc_syndrome(d31):
    r = 0
    for i in range(30, -1, -1):
        fb = r >> 9 & 1
        r = (r << 1 | d31 >> i & 1) & 1023
        if fb:
            r ^= 873
    return r

def _build_syndrome_lut():
    return {_calc_syndrome(1 << p): p for p in range(31)}

def _d8(bb, n, nco_phase, nco_step, pll_int, pk_mx, pk_mn, lh, lc, hyst=0.03):
    import array as _arr
    bits = _arr.array('b')
    ba = bits.append
    _ph = nco_phase
    _int = pll_int
    _mx, _mn = (pk_mx, pk_mn)
    _lh, _lc = (lh, lc)
    MD = nco_step * 0.02
    step = nco_step
    bbl = bb.tolist() if hasattr(bb, 'tolist') else list(bb)
    for i in range(n):
        val = bbl[i]
        ha = max(1e-06, (_mx - _mn) * 0.5)
        if val > _mx:
            _mx = val
        else:
            _mx -= ha * 0.0005
        if val < _mn:
            _mn = val
        else:
            _mn += ha * 0.0005
        th = (_mx + _mn) * 0.5
        amp = max(1e-06, _mx - _mn)
        h = max(0.006, min(hyst, 0.15 * amp))
        if val > th + h:
            hb = 1
        elif val < th - h:
            hb = 0
        else:
            hb = _lh
        if hb != _lh:
            err = _ph
            if err > 0.5:
                err -= 1.0
            _int += 0.005 * err
            if _int > MD:
                _int = MD
            elif _int < -MD:
                _int = -MD
            _ph -= 0.1 * err + _int
        _lh = hb
        _ph += step
        if _ph > 0.5:
            if _lc == 0:
                ba(1 - hb)
            _lc = 1
        else:
            _lc = 0
        if _ph >= 1.0:
            _ph -= 1.0
    return (bits.tolist(), _ph, _int, _mx, _mn, _lh, _lc)

class LBJRealtimeDecoder:

    def __init__(self, fs=48000, baud_rate=1200, arrival_estimator=None):
        self.arrival_estimator = arrival_estimator
        self.syndrome_lut = _build_syndrome_lut()
        taps = ssig.firwin(numtaps=31, cutoff=1200, fs=fs, window=('gaussian', 7.0))
        self.b_smooth = taps / np.sum(taps)
        self.zi_smooth = ssig.lfilter_zi(self.b_smooth, 1.0) * 0.0
        self.nco_phase, self.nco_step = (0.0, baud_rate / fs)
        self.pll_integrator = 0.0
        self.peak_max = self.peak_min = 0.0
        self.last_hard_bit = self.last_bit_clk = 0
        self.SYNC_STD, self.SYNC_INV, self.IDLE_WORD = (2094142936, 2200824359, 2055848343)
        self.state = self.shift_reg = self.cw_bit_count = 0
        self.pol = 1
        self.word_count = self.frame_pos = 0
        self.in_message = False
        self.current_addr = self.current_func = 0
        self.current_msg_cws_int = []
        self.hunt_timeout = 0
        self.sessions = {}
        self.last_train = None
        self.last_warn_time = 0.0
        self.last_intent_time = 0.0
        self.current_msg_has_error = False
        self.locos = {1: '解放', 3: '前进', 5: '建设', 6: 'KD7', 55: '蓝箭控车', 81: '东风21', 101: '东风', 102: '东风2', 103: '东风3', 104: '东风4', 105: '东风4客', 106: '东风4C', 107: '东风5', 108: '东风5宽', 109: '东风6', 110: '东风7', 111: '东风8', 112: '东风9', 113: '东风10', 114: '东方红1', 115: '东方红2', 116: '东方红3', 117: '东方红5', 118: '北京', 119: '北京宽', 120: 'ND2', 121: 'ND3', 122: 'ND4', 123: 'ND5', 124: 'NY5', 125: 'NY6', 126: 'NY7', 127: '轻油', 128: '东方红21', 129: '东风7B', 130: '东风5S', 131: '东风7C', 132: '东风7S', 133: '工矿1', 134: '工矿1F', 135: '东风4E', 136: '东风7D', 137: '工矿1A', 138: '东风11', 139: '天安', 140: '东风10F', 141: '东风4D', 142: '东风8B', 143: '东风12', 144: '东风7E', 145: 'NYJ1', 146: 'NZJ1', 147: 'NZJ2', 148: '东风4DJ', 149: '新曙光', 150: '神州', 151: 'NJ2', 152: '东风7G', 153: 'NDJ3', 157: 'FXN3D', 158: '东风11G', 160: 'HXN3', 161: 'HXN5', 162: 'HXN3B', 163: 'HXN5B', 167: 'FXN3B', 170: 'FXN5C', 171: 'FXN3-J', 201: '8G', 202: '8K', 203: '6G', 204: '6K', 205: '韶山1', 206: '韶山3', 207: '韶山4', 208: '韶山5', 209: '韶山6', 210: '韶山3B', 211: '韶山7', 212: '韶山8', 213: '韶山7B', 214: '韶山7C', 215: '韶山6B', 216: '韶山9', 217: '韶山7D', 218: 'DJ熊猫', 219: 'DJ1', 220: 'DJ2', 221: 'DJF', 222: '蓝箭动车', 223: '先锋号', 224: '韶山7E', 225: '韶山4G', 226: '韶山3C', 228: '天梭', 229: 'DJ4和谐', 230: 'KTT', 231: 'HXD1', 232: 'HXD2', 233: 'HXD3', 234: 'HXD1B', 235: 'HXD2B', 236: 'HXD3B', 237: 'HXD1C', 238: 'HXD2C', 239: 'HXD3C', 240: 'HXD1D', 241: 'HXD2D', 242: 'HXD3D', 243: 'FXD1B', 244: 'FXD2B', 245: 'FXD1', 246: 'FXD3', 247: 'FXD1-J', 248: 'FXD3-J', 249: 'KZ25TA', 251: 'KZ25TB', 252: 'HXD1D-J', 254: 'FXD1H', 299: '雪域神州', 300: 'CRH1', 301: 'CRH2', 302: 'CRH3', 304: 'CRH5', 305: 'CRH380A', 306: 'CRH380B', 307: 'CRH380C', 308: 'CRH380D', 309: 'CRH6A', 310: 'CR400AF', 311: 'CR400BF', 312: 'CR300AF', 313: 'CR300BF', 314: 'CRH2E', 315: 'CRH6F', 329: 'CJ1', 330: 'CJ2', 331: 'CJ3', 332: 'CJ4', 333: 'CJ5', 334: 'CJ6'}

    def reset_dpll_soft(self):
        self.nco_phase = 0.0
        self.pll_integrator = 0.0
        self.peak_max = 0.0
        self.peak_min = 0.0
        self.last_hard_bit = 0
        self.last_bit_clk = 0

    def reset_receiver_state(self):
        self.reset_dpll_soft()
        self.zi_smooth = ssig.lfilter_zi(self.b_smooth, 1.0) * 0.0
        self.state = 0
        self.shift_reg = 0
        self.cw_bit_count = 0
        self.pol = 1
        self.word_count = 0
        self.frame_pos = 0
        self.in_message = False
        self.current_msg_cws_int = []
        self.current_msg_has_error = False
        self.hunt_timeout = 0

    @staticmethod
    def bcd_to_hex_char(c):
        return {'*': 'A', 'U': 'B', ' ': 'C', '-': 'D', ')': 'E', '(': 'F'}.get(c, c)

    def extract_bcd(self, cw_list):
        chars = []
        for cw in cw_list:
            data20 = cw >> 11 & 1048575
            for n in range(5):
                val = data20 >> 16 - n * 4 & 15
                rev = (val & 1) << 3 | (val & 2) << 1 | (val & 4) >> 1 | (val & 8) >> 3
                chars.append('0123456789*U -)('[rev])
        return ''.join(chars)

    def bch_decode_fast(self, cw_val):
        data31 = cw_val >> 1 & 2147483647
        syn = _calc_syndrome(data31)
        if syn == 0:
            return (cw_val, True)
        fp = self.syndrome_lut.get(syn)
        if fp is not None:
            return ((data31 ^ 1 << fp) << 1 | cw_val & 1, True)
        return (cw_val, False)

    def send_local_intent(self, train, direction, speed, position, loco, loco_code, route, category):
        if os.name == 'nt' or train in ('----', ''):
            return
        ct = time.time()
        if ct - self.last_intent_time < 3.0:
            return
        self.last_intent_time = ct

        def _s():
            try:
                subprocess.run(['am', 'broadcast', '--user', '0', '-a', 'com.train.alert', '-p', 'com.arlosoft.macrodroid', '-e', 'train', str(train), '-e', 'dir', str(direction), '-e', 'speed', str(speed), '-e', 'pos', str(position), '-e', 'loco', str(loco), '-e', 'code', str(loco_code), '-e', 'route', str(route), '-e', 'cat', str(category)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2)
            except:
                pass
        try:
            threading.Thread(target=_s).start()
        except:
            pass

    def emit_warning(self, func):
        now = time.time()
        if now - self.last_warn_time < 1.0:
            return
        self.last_warn_time = now
        d = '下行' if func == 1 else '上行' if func == 3 else '未知'
        _g2['warning'] = f'\x1b[5m\x1b[41m\x1b[97m ⚠ 探测到{d}信号 干扰严重 \x1b[0m'
        _g2['warning_time'] = now
        self.send_local_intent(train='⚠弱信号', direction=d, speed='---', position='---.-', loco='信号受干扰', loco_code='---', route='----', category='错包预警')

    def process_chunk(self, audio_chunk):
        bb, self.zi_smooth = ssig.lfilter(self.b_smooth, [1.0], audio_chunk, zi=self.zi_smooth)
        res = _d8(bb, len(bb), self.nco_phase, self.nco_step, self.pll_integrator, self.peak_max, self.peak_min, self.last_hard_bit, self.last_bit_clk)
        bits, self.nco_phase, self.pll_integrator, self.peak_max, self.peak_min, self.last_hard_bit, self.last_bit_clk = res
        for bit in bits:
            self.process_bit_streaming(bit)

    def process_bit_streaming(self, bit):
        self.shift_reg = (self.shift_reg << 1 | bit) & 4294967295
        if self.state == 0:
            if _popcount32(self.shift_reg ^ self.SYNC_STD) <= 2:
                self.pol, self.state = (1, 1)
                self.word_count = self.frame_pos = self.cw_bit_count = self.hunt_timeout = 0
            elif _popcount32(self.shift_reg ^ self.SYNC_INV) <= 2:
                self.pol, self.state = (-1, 1)
                self.word_count = self.frame_pos = self.cw_bit_count = self.hunt_timeout = 0
            if self.in_message:
                self.hunt_timeout += 1
                if self.hunt_timeout > 64:
                    self.trigger_lbj_parse()
        elif self.state == 1:
            self.cw_bit_count += 1
            if self.cw_bit_count == 32:
                self.cw_bit_count = 0
                raw = self.shift_reg if self.pol == 1 else ~self.shift_reg & 4294967295
                corrected, is_bch_valid = self.bch_decode_fast(raw)
                self.frame_pos += 1
                self.word_count += 1
                if _popcount32(corrected ^ self.SYNC_STD) <= 2:
                    self.word_count = self.frame_pos = 0
                elif _popcount32(corrected ^ self.IDLE_WORD) <= 2:
                    if self.in_message:
                        self.trigger_lbj_parse()
                elif corrected >> 31 == 0:
                    if self.in_message:
                        self.trigger_lbj_parse()
                    self.current_func = corrected >> 11 & 3
                    self.current_addr = (corrected >> 13 & 262143) * 8 + (self.frame_pos - 1) // 2
                    self.current_msg_cws_int = []
                    self.in_message = True
                    self.current_msg_has_error = not is_bch_valid
                elif corrected >> 31 == 1:
                    if self.in_message:
                        self.current_msg_cws_int.append(corrected)
                        if not is_bch_valid:
                            self.current_msg_has_error = True
                if self.word_count >= 16:
                    self.state = 0

    def trigger_lbj_parse(self):
        self.in_message = False
        if self.current_msg_cws_int:
            if self.current_msg_has_error:
                if _g0['show_err_warn']:
                    self.emit_warning(self.current_func)
                if _g0['strict_filter']:
                    self.current_msg_cws_int = []
                    self.current_msg_has_error = False
                    return
            valid_for_eta = not self.current_msg_has_error
            self.decode_lbj(self.extract_bcd(self.current_msg_cws_int), valid_for_eta=valid_for_eta)
            self.current_msg_cws_int = []
            self.current_msg_has_error = False

    def check_filter_and_update_ui(self):
        hit = False
        if _g0['keywords']:
            tu = _g1['train'].replace(' ', '').upper()
            lu = _g1['loco'].replace(' ', '').upper()
            for kw in _g0['keywords']:
                ku = kw.replace(' ', '').upper()
                if ku and ku in lu:
                    hit = True
                    break
                if ku and tu:
                    if ku == tu:
                        hit = True
                        break
                    kw_n = re.search('\\d+', ku)
                    tr_n = re.search('\\d+', tu)
                    if kw_n and tr_n and (kw_n.group(0) == tr_n.group(0)):
                        if ku.isdigit():
                            hit = True
                            break
                        elif not re.sub('\\d+', '', tu):
                            hit = True
                            break
        if _g0['keywords'] and _g0['filter_mode'] == 'strict' and (not hit):
            return False
        _g2.update(_g1)
        _g2['category'] = _u5(_g2['train'], _g2['is_detailed'])
        _g2['is_hit'] = hit
        valid = _g2['train'] != '----'
        want = not _g0['keywords'] or _g2['is_hit']
        if valid and want:
            self.send_local_intent(train=_g2['train'], direction=_g2['direction'], speed=_g2['speed'], position=_g2['position'], loco=_g2['loco'], loco_code=_g2['loco_code'], route=_g2['route'], category=_g2['category'])
        return True

    def decode_lbj(self, bcd, valid_for_eta=True):
        addr, func = (self.current_addr, self.current_func)
        ct = time.time()
        if addr == 1234008 or (addr in (1233999, 1234000) and func == 0 and (len(bcd) == 5) and (bcd[0] in ('-', '*')) and (bcd[4] != '-')):
            return
        is_short = addr in (1233999, 1234000) and len(bcd) >= 15
        is_merged = addr in (1233999, 1234000, 1234002) and len(bcd) >= 65
        is_standalone = addr in (1234001, 1234002) and len(bcd) < 65
        base_train = None
        if is_short:
            raw = bcd[0:5].strip()
            if _g0['strict_filter']:
                base_train = raw if re.match('^[A-Za-z0-9]+$', raw) else '----'
            else:
                base_train = raw if '*' not in raw and '-' not in raw else '----'
            self.last_train = base_train
            direction = '下行' if func == 1 else '上行' if func == 3 else f'未知({func})'
            rs = bcd[6:9].replace(' ', '0').replace('U', '0').replace('*', '0')
            speed = str(int(rs)) if rs.isdigit() and 0 <= int(rs) <= 400 else '---'
            pr = bcd[10:15]
            if any((c in pr for c in ['*', '-', 'X', 'U'])):
                position = '---.-'
            else:
                ps = pr.replace(' ', '0')
                position = f'{ps[0:4]}.{ps[4]}' if ps.isdigit() else '---.-'
            if base_train not in self.sessions:
                self.sessions[base_train] = {'base_train': base_train, 'prefix': '', 'direction': direction, 'speed': speed, 'position': position, 'loco': '----', 'loco_code': '---', 'route': '----', 'route_valid': False, 'is_detailed': False}
            else:
                s = self.sessions[base_train]
                if direction != '未知' and (not direction.startswith('未知')):
                    s['direction'] = direction
                if speed != '---':
                    s['speed'] = speed
                if position != '---.-':
                    s['position'] = position
            self.sessions[base_train]['timestamp'] = ct
        if is_merged or is_standalone:
            tid = base_train if is_merged else self.last_train
            if tid and tid in self.sessions:
                if is_standalone and ct - self.sessions[tid]['timestamp'] > 2.0:
                    pass
                else:
                    buf = bcd[-50:]
                    ih = ''.join((self.bcd_to_hex_char(c) for c in buf))
                    lc = ''
                    if len(ih) >= 6:
                        try:
                            c1, c2 = (int(ih[0:2], 16), int(ih[2:4], 16))
                            if _g0['strict_filter']:
                                if c1 in range(48, 58) or c1 in range(65, 91):
                                    lc += chr(c1)
                                if c2 in range(48, 58) or c2 in range(65, 91):
                                    lc += chr(c2)
                            else:
                                if 31 < c1 < 127 and c1 not in (34, 44):
                                    lc += chr(c1)
                                if 31 < c2 < 127 and c2 not in (34, 44):
                                    lc += chr(c2)
                        except:
                            pass
                    lc = lc.replace(' ', '').strip()
                    lr = buf[4:12] if len(buf) >= 12 else ''
                    lm, lk = ('----', '---')
                    if len(lr) >= 3:
                        cp, np_ = (lr[0:3], lr[3:].strip())
                        if cp.isdigit():
                            lk = cp
                            ti = int(cp)
                            ln = self.locos.get(ti, '未知型号')
                            if _g0['strict_filter']:
                                if not np_.isdigit():
                                    np_ = '----'
                            elif any((c in np_ for c in ['*', '-', 'X', ' '])) or not np_:
                                np_ = '----'
                            lm = f'{ln}-{np_}'
                    ru = '----'
                    if len(ih) >= 30:
                        try:
                            rb = bytes.fromhex(ih[14:30])
                            dec = rb.decode('gbk', errors='ignore').replace('\x00', '').strip()
                            if _g0['strict_filter']:
                                if dec and re.match('^[\\u4e00-\\u9fa5A-Za-z0-9\\-\\s]+$', dec):
                                    ru = dec
                            elif dec and re.search('[\\u4e00-\\u9fa5a-zA-Z0-9]', dec):
                                ru = dec
                        except:
                            pass
                    if lc:
                        self.sessions[tid]['prefix'] = lc
                    if lm != '----':
                        self.sessions[tid]['loco'] = lm
                        self.sessions[tid]['loco_code'] = lk
                    if valid_for_eta and ru != '----' and _A0._a4(ru):
                        self.sessions[tid]['route'] = ru
                        self.sessions[tid]['route_valid'] = True
                    self.sessions[tid]['is_detailed'] = True
                    self.sessions[tid]['timestamp'] = ct
        aid = base_train if is_short else self.last_train
        if aid and aid in self.sessions:
            s = self.sessions[aid]
            ft = f"{s['prefix']}{s['base_train']}".replace(' ', '').strip()
            _g1.update({'train': ft, 'direction': s['direction'], 'speed': s['speed'], 'position': s['position'], 'loco': s['loco'], 'loco_code': s['loco_code'], 'route': s['route'], 'route_valid': bool(s.get('route_valid', False)), 'is_detailed': s['is_detailed']})
            updated_ui = self.check_filter_and_update_ui()
            if updated_ui and self.arrival_estimator is not None:
                route_ok = bool(valid_for_eta and s.get('route_valid', False))
                self.arrival_estimator._ad(good_data=route_ok, now=ct)
        for k in [k for k, v in self.sessions.items() if ct - v['timestamp'] > 10]:
            del self.sessions[k]
            if self.last_train == k:
                self.last_train = None

def _u6(db_array, height=6):
    db_max = np.max(db_array)
    db_min = np.percentile(db_array, 10)
    if db_max - db_min < 20:
        db_max = db_min + 20
    lines = []
    db_range = db_max - db_min
    cols_h = [int(max(0.0, min(1.0, (db - db_min) / db_range)) * height * 8) for db in db_array]
    chars = [' ', '▂', '▃', '▄', '▅', '▆', '▇', '█']
    for r in range(height - 1, -1, -1):
        row_db = db_min + (r + 1) * (db_range / height)
        row_str = f'\x1b[90m{int(row_db):4d}│\x1b[0m'
        for c in range(_g3):
            sh = cols_h[c] - r * 8
            if sh >= 8:
                row_str += '\x1b[92m█\x1b[0m'
            elif sh <= 0:
                row_str += ' '
            else:
                row_str += f'\x1b[92m{chars[sh]}\x1b[0m'
        lines.append(row_str)
    lines.append('\x1b[90m     └' + '─' * 15 + '┴' + '─' * 16 + '┘\x1b[0m')
    fc_mhz = _g2.get('freq', 0) / 1000000.0
    gain_v = _g2.get('gain', 0)
    ppm_v = _g2.get('ppm', 0)
    sr_k = _g2.get('sample_rate_k', 240)
    rssi_v = _g2.get('rssi', -140)
    cs_v = _g2.get('cs_threshold', -138)
    lbl = f'▲ {fc_mhz:.4f}M G:{gain_v:.1f} P:{ppm_v} S:{sr_k}k'
    afc_v = _g2.get('afc_hz', 0.0)
    afc_err_v = _g2.get('afc_err_hz', 0.0)
    afc_s = _g2.get('afc_score', 0.0)
    gate_s = _g2.get('rssi_gate', 'OFF')
    hold_ms = _g2.get('rssi_hold_ms', 0.0)
    rssi_lbl = f'RSSI:{rssi_v:.0f} 阈:{cs_v:.0f} RX:{gate_s} H:{hold_ms:.0f}'
    afc_lbl = f'FERR:{afc_err_v:+.0f} AFC:{afc_v:+.0f} S:{afc_s:.2f}'
    lines.append(f'\x1b[96m {lbl}\x1b[0m')
    if rssi_v > cs_v:
        lines.append(f'\x1b[92m {rssi_lbl}\x1b[0m')
    else:
        lines.append(f'\x1b[90m {rssi_lbl}\x1b[0m')
    if abs(afc_err_v) > 50 or abs(afc_v) > 50:
        lines.append(f'\x1b[96m {afc_lbl}\x1b[0m')
    else:
        lines.append(f'\x1b[90m {afc_lbl}\x1b[0m')
    return lines

def _u7(src, frontend=None, rssi_gate=None):
    sys.stdout.write('\x1b[2J\x1b[?25l')
    sys.stdout.flush()
    while _g0['running']:
        if _g0['input_mode']:
            time.sleep(0.1)
            continue
        if _g0['show_help']:
            b_s = '\x1b[92m开启\x1b[0m' if _g0['strict_filter'] else '\x1b[91m关闭\x1b[0m'
            w_s = '\x1b[92m开启\x1b[0m' if _g0['show_err_warn'] else '\x1b[91m关闭\x1b[0m'
            m_s = '\x1b[92m严格(仅显示命中)\x1b[0m' if _g0['filter_mode'] == 'strict' else '\x1b[93m高亮\x1b[0m'
            lines = ['\x1b[96m' + '=' * 38 + '\x1b[0m', '\x1b[93m 解码器菜单与快捷键控制\x1b[0m', '\x1b[96m' + '=' * 38 + '\x1b[0m', '', f'\x1b[97m [B] 错包拦截 : {b_s}', f'\x1b[97m [W] 干扰预警 : {w_s}', f'\x1b[97m [M] 过滤模式 : {m_s}', '', '\x1b[97m [F] 设置关注的车次/机车\x1b[0m', '\x1b[97m [T] 直接输入更改频率\x1b[0m', '\x1b[97m [G] 设置增益 (dB)\x1b[0m', '\x1b[97m [P] 设置时钟PPM校正\x1b[0m', '\x1b[97m [R] 设置RSSI接收门控阈值\x1b[0m', '\x1b[97m [K] 设置当前线路公里标\x1b[0m', '\x1b[97m [C] 清空当前仪表盘数据\x1b[0m', '\x1b[97m [H] 关闭菜单返回主界面\x1b[0m', '\x1b[97m [Q] 安全退出程序\x1b[0m', '', '\x1b[90m' + '─' * 38 + '\x1b[0m']
        else:
            ct = time.strftime('%H:%M:%S')
            kw_str = ','.join(_g0['keywords']) if _g0['keywords'] else '无'
            hit = _g2['is_hit'] and bool(_g0['keywords'])
            vc = '\x1b[91m' if hit else '\x1b[97m'
            dd = _g2['direction']
            ds = f'\x1b[44m {dd} \x1b[0m' if dd == '上行' else f'\x1b[42m {dd} \x1b[0m' if dd == '下行' else dd
            v_train = _u3(_g2['train'], 10)
            spd_val = _g2['speed']
            v_speed = _u3(f'{spd_val}km/h' if spd_val != '---' else '---km/h', 10)
            v_loco = _u3(_g2['loco'], 10)
            v_route = _u3(_g2['route'], 10)
            pos_val = _g2['position']
            v_pos = _u3(f'{pos_val}KM' if pos_val != '---.-' else '---.-KM', 10)
            v_code = _u3(_g2['loco_code'], 8)
            v_cat = _u3(_g2['category'], 10)
            kw_disp = _u3(kw_str, 24)
            lines = [f'\x1b[93m{ct} 过滤:{kw_disp}\x1b[0m']
            w = _g2.get('warning', '')
            wt = _g2.get('warning_time', 0)
            if time.time() - wt < 2 and w:
                lines.append(_u3(w, 38))
            lines.append(f'\x1b[92m车次:\x1b[0m {vc}{v_train}\x1b[0m \x1b[92m方向:\x1b[0m {vc}{ds}\x1b[0m')
            lines.append(f'\x1b[92m速度:\x1b[0m {vc}{v_speed}\x1b[0m \x1b[92m公里:\x1b[0m {vc}{v_pos}\x1b[0m')
            eta_sec = _g2.get('eta_seconds')
            eta_time = _g2.get('eta_time', '--:--:--')
            eta_dist = _g2.get('eta_distance_km')
            eta_status = _u3(_g2.get('eta_status', '未设置位置'), 10)
            route_km_text = _g2.get('current_route_km_text', '---')
            eta_sec_txt = '---' if eta_sec is None else f'{int(round(eta_sec))}s'
            eta_time_txt = eta_time if eta_sec is not None else '--:--:--'
            eta_dist_txt = '---' if eta_dist is None else f'{eta_dist:.1f}km'
            v_eta = _u3(f'{eta_sec_txt} {eta_time_txt}', 15)
            v_mykm = _u3(route_km_text, 10)
            lines.append(f'\x1b[92m到达:\x1b[0m {vc}{v_eta}\x1b[0m \x1b[92m本站:\x1b[0m {vc}{v_mykm}\x1b[0m')
            lines.append(f'\x1b[92m距离:\x1b[0m {vc}{_u3(eta_dist_txt, 10)}\x1b[0m \x1b[92m状态:\x1b[0m {vc}{eta_status}\x1b[0m')
            lines.append(f'\x1b[92m机车:\x1b[0m {vc}{v_loco}\x1b[0m \x1b[92m代号:\x1b[0m {vc}{v_code}\x1b[0m')
            lines.append(f'\x1b[92m线路:\x1b[0m {vc}{v_route}\x1b[0m \x1b[92m类别:\x1b[0m {vc}{v_cat}\x1b[0m')
            lines.append('\x1b[90m' + '─' * 38 + '\x1b[0m')
            spec_lines = _u6(_g4['smoothed'], height=4)
            lines.extend(spec_lines)
            lines.append('\x1b[90m' + '─' * 38 + '\x1b[0m')
            lines.append('\x1b[93m[T]调频 [G]增益 [P]PPM [H]菜单\x1b[0m')
            lines.append('\x1b[93m[R]阈值 [K]位置 [F]过滤\x1b[0m')
            lines.append('\x1b[93m[C]清屏 [Q]退出\x1b[0m')
        out = '\x1b[H' + '\n'.join((l + '\x1b[K' for l in lines)) + '\x1b[0J'
        sys.stdout.write(out)
        sys.stdout.flush()
        key = _u4()
        if key == 'q':
            _g0['running'] = False
            break
        elif key == 'h':
            _g0['show_help'] = not _g0['show_help']
            sys.stdout.write('\x1b[2J')
            sys.stdout.flush()
        elif key == 'b':
            _g0['strict_filter'] = not _g0['strict_filter']
        elif key == 'w':
            _g0['show_err_warn'] = not _g0['show_err_warn']
        elif key == 'm':
            _g0['filter_mode'] = 'strict' if _g0['filter_mode'] == 'highlight' else 'highlight'
            if _g0['filter_mode'] == 'strict' and (not _g2['is_hit']):
                _g2.update({'train': '----', 'direction': '未知', 'speed': '---', 'position': '---.-', 'loco': '----', 'loco_code': '---', 'route': '----', 'category': '等待命中...'})
        elif key == 't':
            _g0['input_mode'] = True
            sys.stdout.write('\x1b[2J\x1b[H\x1b[?25h')
            sys.stdout.flush()
            try:
                nk = input('\n\x1b[93m频率 (MHz, 如 821.2375): \x1b[0m')
                if nk.strip():
                    new_freq = float(nk.strip()) * 1000000.0
                    src._af(new_freq)
                    _g2['freq'] = new_freq
                    if frontend is not None:
                        frontend.reset_afc()
                    if rssi_gate is not None:
                        rssi_gate.reset()
            except:
                pass
            sys.stdout.write('\x1b[2J\x1b[?25l')
            sys.stdout.flush()
            _g0['input_mode'] = False
        elif key == 'g':
            _g0['input_mode'] = True
            sys.stdout.write('\x1b[2J\x1b[H\x1b[?25h')
            sys.stdout.flush()
            try:
                gl = ' '.join((f'{g:.1f}' for g in R820T_GAINS))
                print(f'\n\x1b[96m可用增益档位 (dB):\x1b[0m\n\x1b[97m{gl}\x1b[0m')
                print(f"\x1b[90m当前: {_g2.get('gain', HW_GAIN_DB):.1f} dB\x1b[0m")
                nk = input(f'\x1b[93m输入增益 dB: \x1b[0m')
                if nk.strip():
                    actual = src._ag(float(nk.strip()))
                    _g2['gain'] = actual
            except:
                pass
            sys.stdout.write('\x1b[2J\x1b[?25l')
            sys.stdout.flush()
            _g0['input_mode'] = False
        elif key == 'p':
            _g0['input_mode'] = True
            sys.stdout.write('\x1b[2J\x1b[H\x1b[?25h')
            sys.stdout.flush()
            try:
                print(f"\n\x1b[90m当前 PPM: {_g2.get('ppm', PPM)}\x1b[0m")
                nk = input('\x1b[93m输入 PPM 校正值: \x1b[0m')
                if nk.strip():
                    new_ppm = int(nk.strip())
                    src._ah(new_ppm)
                    _g2['ppm'] = new_ppm
                    if frontend is not None:
                        frontend.reset_afc()
                    if rssi_gate is not None:
                        rssi_gate.reset()
            except:
                pass
            sys.stdout.write('\x1b[2J\x1b[?25l')
            sys.stdout.flush()
            _g0['input_mode'] = False
        elif key == 'r':
            _g0['input_mode'] = True
            sys.stdout.write('\x1b[2J\x1b[H\x1b[?25h')
            sys.stdout.flush()
            try:
                cur = _g2.get('cs_threshold', -138)
                print(f'\n\x1b[90m当前RSSI解调阈值: {cur:.0f} dB\x1b[0m')
                print(f'\x1b[90m(用于RSSI接收门控，不直接参与FSK 0/1判决)\x1b[0m')
                nk = input('\x1b[93m输入新阈值 dB: \x1b[0m')
                if nk.strip():
                    new_th = float(nk.strip())
                    _g2['cs_threshold'] = new_th
                    if rssi_gate is not None:
                        rssi_gate._ae(new_th)
            except:
                pass
            sys.stdout.write('\x1b[2J\x1b[?25l')
            sys.stdout.flush()
            _g0['input_mode'] = False
        elif key == 'k':
            _g0['input_mode'] = True
            sys.stdout.write('\x1b[2J\x1b[H\x1b[?25h')
            sys.stdout.flush()
            try:
                est = _g2.get('_arrival_estimator')
                cur_route = _A0._a0(_g2.get('route'))
                print('\n\x1b[96m线路公里标设置\x1b[0m')
                if est is not None:
                    if est.known_routes:
                        print('\x1b[90m已从可靠报文提取的线路:\x1b[0m')
                        for r in est.known_routes[-10:]:
                            km = est.route_km_map.get(r)
                            km_txt = _A0._a1(km) if km is not None else '未设置'
                            mark = '*' if r == cur_route else ' '
                            print(f' {mark} {r}: {km_txt}')
                    else:
                        print('\x1b[90m尚未从可靠报文中提取到线路。\x1b[0m')
                    if cur_route:
                        cur_km = est.route_km_map.get(cur_route)
                        cur_txt = '未设置' if cur_km is None else _A0._a1(cur_km)
                        print(f'\n\x1b[90m当前线路: {cur_route}  本站: {cur_txt}\x1b[0m')
                        prompt = '输入当前线路本站公里标 xxxx.xKM，留空=清除此线路，或输入 线路=xxxx.xKM: '
                    else:
                        prompt = '输入 线路=xxxx.xKM，例如 京沪线=0123.4KM: '
                    nk = input(f'\x1b[93m{prompt}\x1b[0m')
                    text = nk.strip()
                    if text:
                        if '=' in text:
                            route, val = text.split('=', 1)
                            est._a9(route, val)
                        elif cur_route:
                            est._a9(cur_route, text)
                    elif cur_route:
                        est._aa(cur_route)
            except Exception:
                pass
            sys.stdout.write('\x1b[2J\x1b[?25l')
            sys.stdout.flush()
            _g0['input_mode'] = False
        elif key == 'c':
            _g2.update({'train': '----', 'direction': '未知', 'speed': '---', 'position': '---.-', 'loco': '----', 'loco_code': '---', 'route': '----', 'category': '等待信号...', 'warning': '', 'warning_time': 0, 'eta_seconds': None, 'eta_time': '--:--:--', 'eta_distance_km': None, 'eta_status': '等待信号', 'eta_train': '----', 'eta_route': '----'})
            _g1.update({'train': '----', 'direction': '未知', 'speed': '---', 'position': '---.-', 'loco': '----', 'loco_code': '---', 'route': '----', 'route_valid': False, 'is_detailed': False})
        elif key == 'f':
            _g0['input_mode'] = True
            sys.stdout.write('\x1b[2J\x1b[H\x1b[?25h')
            sys.stdout.flush()
            try:
                nk = input('\n\x1b[93m关注车次 (逗号分隔,留空=全部): \x1b[0m')
                _g0['keywords'] = [k.strip().upper() for k in nk.split(',') if k.strip()] if nk.strip() else []
            except:
                pass
            sys.stdout.write('\x1b[2J\x1b[?25l')
            sys.stdout.flush()
            _g0['input_mode'] = False
        time.sleep(0.05)
    sys.stdout.write('\x1b[2J\x1b[H\x1b[?25h\x1b[0m')
    sys.stdout.flush()

def _u8(src, frontend, decoder, rssi_gate=None, reset_afc_on_release=True):
    while _g0['running']:
        try:
            iq = src.read()
        except RuntimeError as e:
            _g2['warning'] = str(e)
            _g2['warning_time'] = time.time()
            break
        except queue.Empty:
            continue
        except Exception as e:
            _g2['warning'] = f'错误:{e}'
            _g2['warning_time'] = time.time()
            break
        if len(iq) >= 1024:
            n = 1024
            win = np.hanning(n)
            seg = iq[:n] * win
            spec = np.fft.fftshift(np.fft.fft(seg))
            mag = np.abs(spec) / (n * np.mean(win))
            db = 20.0 * np.log10(np.maximum(mag, 1e-12))
            chunk_sz = 1024 // _g3
            pooled = np.array([np.max(db[i * chunk_sz:(i + 1) * chunk_sz]) for i in range(_g3)])
            _g4['smoothed'] = 0.7 * _g4['smoothed'] + 0.3 * pooled
        pcm_float, avg_rssi, rx_active = frontend.process(iq, rssi_gate=rssi_gate)
        _g2['rssi'] = avg_rssi
        _g2['afc_hz'] = frontend.afc.afc_hz
        _g2['afc_err_hz'] = frontend.afc.last_err_hz
        _g2['afc_score'] = frontend.afc.last_score
        if rssi_gate is not None:
            _g2['rssi_gate'] = rssi_gate.state
            _g2['rssi_hold_ms'] = rssi_gate.hold_left_ms
        else:
            _g2['rssi_gate'] = 'BYPASS'
            _g2['rssi_hold_ms'] = 0.0
        if frontend.consume_afc_updated():
            decoder.reset_dpll_soft()
        if rx_active:
            decoder.process_chunk(pcm_float)
        elif rssi_gate is None or rssi_gate.just_deactivated:
            decoder.reset_receiver_state()
            if reset_afc_on_release and frontend.afc.enabled:
                frontend.reset_afc()
                _g2['afc_hz'] = 0.0
                _g2['afc_err_hz'] = 0.0
                _g2['afc_score'] = 0.0

def _u9():
    p = argparse.ArgumentParser(description='SDR-LBJ v15.3c fixed 960 kS/s (AFC + RSSI gate + route ETA)')
    p.add_argument('-f', '--freq', type=float, default=821.2375, help='频率 MHz')
    p.add_argument('-g', '--gain', type=float, default=HW_GAIN_DB, help='增益 dB')
    p.add_argument('-p', '--ppm', type=int, default=PPM, help='PPM 校正')
    p.add_argument('--dc-offset', type=float, default=DEFAULT_DC_OFFSET_HZ / 1000.0, help='DC 避让偏移 kHz，固定 960k 下默认 50')
    p.add_argument('--no-dc-offset', action='store_true', help='关闭 DC 避让，不推荐')
    p.add_argument('--bw', type=float, default=DEFAULT_BW_KHZ, help='信道带宽 kHz，默认 19.5')
    p.add_argument('--cs-threshold', type=float, default=DEFAULT_RSSI_THRESHOLD_DB, help='RSSI 接收门控打开阈值 dB，默认 -45；不直接参与 FSK 0/1 判决')
    p.add_argument('--no-rssi-gate', action='store_true', help='关闭 RSSI 接收门控；关闭后持续解码，便于对比调试')
    p.add_argument('--rssi-hyst', type=float, default=DEFAULT_RSSI_HYST_DB, help='RSSI 门控释放迟滞 dB，默认 4，即 OFF=threshold-4dB')
    p.add_argument('--rssi-hold-ms', type=float, default=DEFAULT_RSSI_HOLD_MS, help='RSSI 低于释放门限后的接收保持时间 ms，默认 700')
    p.add_argument('--rssi-confirm-blocks', type=int, default=1, help='RSSI 连续超过门限多少块后打开接收，默认 1')
    p.add_argument('--rssi-offset', type=float, default=0.0, help='RSSI 显示偏移 dB')
    p.add_argument('--afc-off', action='store_true', help='关闭导前码 AFC，仅保留基础 DDC')
    p.add_argument('--afc-max', type=float, default=DEFAULT_AFC_MAX_HZ, help='AFC 最大修正范围 Hz，默认 ±1500')
    p.add_argument('--afc-gain', type=float, default=0.45, help='AFC 环路增益，默认 0.45，建议 0.25~0.6')
    p.add_argument('--keep-afc-after-packet', action='store_true', help='RSSI门控释放后保留AFC补偿；默认每次接收完成后复位AFC，避免补偿累积带偏下一包')
    p.add_argument('--my-km', type=float, default=None, help='全局默认当前位置公里标 km；多线路建议使用 --route-km 或运行时按 K 按线路设置')
    p.add_argument('--route-km', action='append', default=[], help='按线路设置本站公里标，格式: 线路=xxxx.xKM；可重复或用逗号分隔，例如 --route-km 京沪线=0123.4KM')
    p.add_argument('--eta-max-min', type=float, default=DEFAULT_ETA_MAX_SECONDS / 60.0, help='最大显示到达时间分钟数，超过则显示 ETA过大，默认 360')
    a = p.parse_args()
    sample_rate = RTL_SAMPLE_RATE
    halfband_n = HALFBAND_STAGES
    mid_rate = MID_RATE
    block_size = BLOCK_SIZE
    if a.no_dc_offset:
        dc_hz = 0.0
    else:
        dc_hz = float(a.dc_offset) * 1000.0
        mx = sample_rate / 2.0 - a.bw * 1000.0 / 2.0 - 5000
        if dc_hz > mx:
            print(f'[警告] DC偏移限制到 {mx / 1000.0:.0f} kHz')
            dc_hz = max(0.0, mx)
    fc = a.freq * 1000000.0
    hw_tune = fc - dc_hz
    _g2['freq'] = fc
    _g2['gain'] = min(R820T_GAINS, key=lambda g: abs(g - a.gain))
    _g2['ppm'] = a.ppm
    _g2['sample_rate_k'] = 960
    _g2['cs_threshold'] = a.cs_threshold
    src = _A2(TCP_HOST, TCP_PORT, hw_tune, sample_rate, block_size, dc_offset=dc_hz)
    frontend = _D7(sample_rate, halfband_n, mid_rate, dc_offset=dc_hz, user_offset=0.0, bw=a.bw * 1000.0, rssi_offset=a.rssi_offset, afc_enable=not a.afc_off, afc_max_hz=a.afc_max, afc_gain=a.afc_gain)
    route_km_map = _A0._a3(a.route_km)
    arrival_estimator = _A0(user_km=a.my_km, max_seconds=max(1.0, a.eta_max_min * 60.0), route_km_map=route_km_map)
    _g2['_arrival_estimator'] = arrival_estimator
    decoder = LBJRealtimeDecoder(BASEBAND_RATE, BAUD_RATE, arrival_estimator=arrival_estimator)
    rssi_gate = _A1(on_db=a.cs_threshold, hysteresis_db=a.rssi_hyst, hold_ms=a.rssi_hold_ms, confirm_blocks=a.rssi_confirm_blocks, enabled=not a.no_rssi_gate)
    print('=' * 42)
    print('  SDR-LBJ v15.3c fixed 960k + AFC/RSSI Gate + Route ETA')
    print('=' * 42)
    print(f'  频率: {a.freq:.4f} MHz  调谐: {hw_tune / 1000000.0:.6f} MHz')
    print(f'  采样: 960 kS/s  块长: {block_size} IQ')
    print(f'  链路: {frontend.chain_desc}')
    if dc_hz > 0:
        print(f'  DC避让: +{dc_hz / 1000.0:.0f} kHz DDC')
    else:
        print('  DC避让: 关闭')
    print(f'  信道带宽: {a.bw:.1f} kHz')
    print(f"  AFC: {('关闭' if a.afc_off else f'开启 ±{a.afc_max:.0f} Hz, gain={a.afc_gain:.2f}, release_reset={not a.keep_afc_after_packet}')}")
    print(f"  RSSI门控: {('关闭' if a.no_rssi_gate else f'开启 ON={a.cs_threshold:.0f} dB OFF={a.cs_threshold - a.rssi_hyst:.0f} dB hold={a.rssi_hold_ms:.0f} ms')}")
    print(f'  RSSI偏移: {a.rssi_offset:.0f} dB')
    if route_km_map:
        route_desc = ', '.join((f'{r}={_A0._a1(k)}' for r, k in route_km_map.items()))
    else:
        route_desc = '未设置线路位置，可按K设置'
    print(f'  到达估算: {route_desc}，下行公里标增大，上行公里标减小')
    print('=' * 42)
    src.open()
    t = threading.Thread(target=_u8, args=(src, frontend, decoder, rssi_gate, not a.keep_afc_after_packet), daemon=True)
    t.start()
    try:
        _u7(src, frontend=frontend, rssi_gate=rssi_gate)
    except KeyboardInterrupt:
        _g0['running'] = False
    _g0['running'] = False
    src.close()
    t.join(timeout=2.0)
if __name__ == '__main__':
    _u9()
