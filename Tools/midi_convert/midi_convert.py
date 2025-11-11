#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把 MIDI 直接转成两个 C 数组：
    const float seq_freq[] = { ... };
    const float seq_ms[]   = { ... };
VOID 用 0.0f 频率表示，持续时间按原样保留。
"""
import sys
import mido
from typing import List, Dict

# ---------- 配置 ----------
TRACK_ID        = 0          # 跳过 tempo track，通常主音符在 1
NOTE_SHIFT      = +2         # 升几个八度（+2 = 频率 ×4）
VOID_MS_THRESH  = 0.1        # 小于此值也强制保留，避免丢拍
# --------------------------

# 音符名 -> 频率（基于 A4=440 Hz）
NOTE2FREQ = {
    'C':  16.3516, 'C#': 17.3239, 'D':  18.3540, 'D#': 19.4454,
    'E':  20.6017, 'F':  21.8268, 'F#': 23.1247, 'G':  24.4997,
    'G#': 25.9565, 'A':  27.5000, 'A#': 29.1352, 'B':  30.8677
}

def note2freq(name: str) -> float:
    """把 'C#4' 转成频率，支持升降八度"""
    name = name.strip()
    if name == 'VOID':
        return 0.0
    octave = int(name[-1])
    pitch  = name[:-1]
    f = NOTE2FREQ[pitch] * (2 ** octave)
    f *= (2 ** NOTE_SHIFT)
    return f

def extract(midi_file: str) -> List[Dict[str, float]]:
    mid = mido.MidiFile(midi_file)
    if TRACK_ID >= len(mid.tracks):
        sys.exit(f'ERR: track {TRACK_ID} not exist')
    track = mid.tracks[TRACK_ID]

    ticks_per_beat = mid.ticks_per_beat
    tempo = 500000          # 默认 120 BPM
    cur_tick = 0
    active = {}             # note -> start_tick
    notes = []              # [{'name':str, 'start_tick':int, 'end_tick':int}]

    def tick2ms(tick: int) -> float:
        return tick * tempo / ticks_per_beat / 1000.0

    for msg in track:
        cur_tick += msg.time
        if msg.type == 'set_tempo':
            tempo = msg.tempo
        elif msg.type == 'note_on' and msg.velocity > 0:
            active[msg.note] = cur_tick
        elif msg.type == 'note_off' or (msg.type == 'note_on' and msg.velocity == 0):
            if msg.note in active:
                start = active.pop(msg.note)
                notes.append({
                    'name': note_number2name(msg.note),
                    'start_tick': start,
                    'end_tick': cur_tick
                })

    # 按开始时间排序
    notes.sort(key=lambda x: x['start_tick'])

    # 生成连贯序列（插入 VOID）
    coherent = []
    expect_tick = 0
    for e in notes:
        # 先补 VOID
        gap = e['start_tick'] - expect_tick
        if gap > 0:
            gap_ms = tick2ms(gap)
            if gap_ms >= VOID_MS_THRESH:
                coherent.append({'name': 'VOID', 'dur_ms': gap_ms})
        # 再补当前音符
        dur_ms = tick2ms(e['end_tick'] - e['start_tick'])
        coherent.append({'name': e['name'], 'dur_ms': dur_ms})
        expect_tick = e['end_tick']

    return coherent

def note_number2name(num: int) -> str:
    names = ['C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B']
    octave = (num // 12) - 1
    return names[num % 12] + str(octave)

def emit_c_array(seq: List[Dict[str, float]]):
    n = len(seq)
    print(f'const float seq_freq[{n}] = {{')
    for e in seq:
        print(f'    {note2freq(e["name"]):9.2f}f,')
    print('};\n')

    print(f'const float seq_ms[{n}] = {{')
    for e in seq:
        print(f'    {e["dur_ms"]:9.2f}f,')
    print('};\n')

def main():
    if len(sys.argv) != 2:
        sys.exit('Usage: python midi_convert.py file.mid > output.inc')
    seq = extract(sys.argv[1])
    emit_c_array(seq)

if __name__ == '__main__':
    main()