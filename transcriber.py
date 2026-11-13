import argparse
import math
import platform
import re
import subprocess
import sys
from typing import Optional


COLORS = {"red", "green", "blue"}
COLOR_PATTERN = re.compile(r"\b(red|green|blue)\b")


def detect_platform():
    """
    Detect the current platform (Raspberry Pi or other).
    """
    if "arm" in platform.uname().machine:
        return "rpi"
    return "computer"


def init_whisper():
    print("Loading Whisper model...")
    import whisper

    whisper_model = whisper.load_model("tiny")  # Use "tiny" for better performance on limited hardware
    return whisper_model


def record_audio_in_memory(audio_interface, duration=5, rate=16000, chunk=1024):
    """
    Record audio in memory using PyAudio.
    """
    import numpy as np
    import pyaudio

    print("Recording... Speak now.")
    audio_format = pyaudio.paInt16
    channels = 1

    stream = audio_interface.open(format=audio_format,
                                  channels=channels,
                                  rate=rate,
                                  input=True,
                                  frames_per_buffer=chunk)
    try:
        frames = [
            stream.read(chunk)
            for _ in range(int(rate / chunk * duration))
        ]
    finally:
        stream.stop_stream()
        stream.close()

    audio_data = np.frombuffer(b"".join(frames), dtype=np.int16)
    print("Recording complete.")
    audio_data = audio_data.astype(np.float32) / 32768.0
    return audio_data


def transcribe_audio_in_memory(whisper_model, audio_data, rate=16000):
    """
    Transcribe audio data from memory using Whisper.
    """
    print("Transcribing...")
    result = whisper_model.transcribe(audio_data, fp16=False, language="en")
    return result["text"]


def whisper_speech_to_text(whisper_model, audio, duration):
    audio_data = record_audio_in_memory(audio, duration=duration)
    return transcribe_audio_in_memory(whisper_model, audio_data)


def parse_grab_command(transcript: str) -> Optional[str]:
    text = transcript.lower()

    if re.search(r"\b(don't|do not|never|not)\b", text):
        return None
    if not re.search(r"\b(pick up|grab|lift)\b", text):
        return None
    if not re.search(r"\b(box|block|cube)\b", text):
        return None

    colors = set(COLOR_PATTERN.findall(text))
    return colors.pop() if len(colors) == 1 else None


def grab_color(color, container):
    if color not in COLORS:
        raise ValueError(f"Unsupported color: {color}")

    subprocess.run(
        [
            "docker", "exec", container, "bash", "-lc",
            'source /opt/ros/jazzy/setup.bash && exec python3 "$@"',
            "color-demo",
            "/workspace/ros2-gazebo/color_demo.py",
            "--color",
            color,
        ],
        check=True,
    )


def main():
    parser = argparse.ArgumentParser(
        description="Recognize spoken color-grab commands and move the arm."
    )
    parser.add_argument("--duration", type=float, default=5.0,
                        help="Seconds to record for each command")
    parser.add_argument("--container", default="senior-design-ros2-gazebo",
                        help="Name of the running ROS 2 Docker container")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration must be a finite number greater than zero")

    import pyaudio

    model = init_whisper()
    microphone = pyaudio.PyAudio()
    try:
        print("Say 'pick up the red cube', 'grab the green block', or 'lift the blue box'.")
        print("Press Ctrl+C to stop.")
        while True:
            transcript = whisper_speech_to_text(model, microphone, args.duration).strip()
            print(f"Recognized: {transcript or '(nothing)'}", flush=True)
            color = parse_grab_command(transcript)
            if color is None:
                print("No valid grab command. Try naming one color and a cube, box, or block.",
                      flush=True)
                continue
            print(f"Grabbing the {color} cube.", flush=True)
            try:
                grab_color(color, args.container)
            except subprocess.CalledProcessError as error:
                print(f"Grab command failed with exit code {error.returncode}.", file=sys.stderr)
                return error.returncode or 1
            except OSError as error:
                print(f"Could not run the grab command: {error}", file=sys.stderr)
                return 1
    except KeyboardInterrupt:
        return 0
    finally:
        microphone.terminate()


if __name__ == "__main__":
    sys.exit(main())