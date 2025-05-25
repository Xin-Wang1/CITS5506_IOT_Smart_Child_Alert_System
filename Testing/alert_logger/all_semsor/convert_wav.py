import wave

input_file = "attention.wav"
output_file = "attention_audio.h"
array_name = "attentionAudio"

with wave.open(input_file, "rb") as w:
    frames = w.readframes(w.getnframes())

with open(output_file, "w") as f:
    f.write(f"const uint8_t {array_name}[] = {{\n")
    for i, b in enumerate(frames):
        f.write(f"{b}, ")
        if i % 20 == 19:
            f.write("\n")
    f.write("\n};\n")
    f.write(f"const int {array_name}Length = sizeof({array_name});\n")
