"""Author two original, understated menu loops; no sampled third-party audio.

32 seconds, stereo PCM16. Overlapping pad chords and sparse bell notes wrap
across the loop boundary so playback does not require a silent tail or cut.
These are intentionally simple composition drafts, not orchestral recordings.
"""

import argparse
import array
import math
import sys
import wave
from pathlib import Path

TRACK_NAMES = ("base-arrival", "departure")
DURATION = 32


def make_loop(name: str, sample_rate: int = 24000) -> array.array:
    if name not in TRACK_NAMES:
        raise ValueError(f"Unknown menu track: {name}")
    frames = DURATION * sample_rate
    left = array.array("f", [0.0]) * frames
    right = array.array("f", [0.0]) * frames

    def note(midi, start, duration, gain, pan, bell=False):
        frequency = 440.0 * 2.0 ** ((midi - 69) / 12.0)
        length = round(duration * sample_rate)
        first = round(start * sample_rate)
        for index in range(length):
            t = index / sample_rate
            phase = 2 * math.pi * frequency * t
            if bell:
                envelope = min(t / .018, 1) * math.exp(-2.4 * t)
                envelope *= min((duration - t) / .3, 1)
                value = math.sin(phase) + .16 * math.sin(phase * 2)
            else:
                envelope = math.sin(math.pi * index / length) ** 2
                value = math.sin(phase) + .12 * math.sin(phase * 2)
            value *= gain * envelope
            at = (first + index) % frames
            left[at] += value * math.sqrt(1 - pan)
            right[at] += value * math.sqrt(pan)

    chords = ((48, 55, 59, 64), (45, 52, 55, 60), (41, 48, 52, 57), (43, 50, 55, 60))
    melody = (76, 79, 74, 72, 71, 76, 69, 74)
    if name == "departure":
        chords = ((50, 57, 60, 65), (46, 53, 57, 62), (48, 55, 60, 64), (45, 52, 57, 60))
        melody = (77, 81, 76, 74, 72, 77, 69, 72)
    for index, chord in enumerate(chords):
        for voice, midi in enumerate(chord):
            note(midi, index * 8 - 2, 12, .025, .2 + voice * .2)
        note(chord[0] - 12, index * 8 - 1, 10, .032, .5)
    for index, midi in enumerate(melody):
        note(midi, index * 4 + 1, 3, .038, .3 if index % 2 else .7, bell=True)
    samples = array.array("h")
    for l_sample, r_sample in zip(left, right):
        samples.extend((round(l_sample * 32767), round(r_sample * 32767)))
    return samples


def write_tracks(directory: Path, sample_rate: int = 24000) -> list[Path]:
    directory.mkdir(parents=True, exist_ok=True)
    outputs = []
    for name in TRACK_NAMES:
        path = directory / f"{name}.wav"
        samples = make_loop(name, sample_rate)
        if sys.byteorder != "little":
            samples.byteswap()
        with wave.open(str(path), "wb") as writer:
            writer.setnchannels(2)
            writer.setsampwidth(2)
            writer.setframerate(sample_rate)
            writer.writeframes(samples.tobytes())
        outputs.append(path)
    return outputs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    for path in write_tracks(args.output):
        print(path)


if __name__ == "__main__":
    main()
