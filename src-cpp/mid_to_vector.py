# pip install pretty_midi
import pretty_midi

MIDI_FILE = "Salgo pa la calle.mid"
MIN_REST_MS = 30
MIN_NOTE_MS = 40

pm = pretty_midi.PrettyMIDI(MIDI_FILE)

# Pick first non-drum instrument (or change index if needed)
inst = next((i for i in pm.instruments if not i.is_drum), pm.instruments[0])

notes = sorted(inst.notes, key=lambda n: n.start)

melody = []
last_end = 0.0

for n in notes:
    # Add rest if there's a gap
    gap_ms = int(round((n.start - last_end) * 1000))
    if gap_ms >= MIN_REST_MS:
        melody.append((0, gap_ms))

    freq = int(round(pretty_midi.note_number_to_hz(n.pitch)))
    dur_ms = int(round((n.end - n.start) * 1000))

    if dur_ms >= MIN_NOTE_MS:
        melody.append((freq, dur_ms))
        last_end = max(last_end, n.end)

# Print C++ initializer
print("std::vector<Note> melody = {")
print(",\n".join([f"    {{{f}, {d}}}" for f, d in melody]))
print("};")