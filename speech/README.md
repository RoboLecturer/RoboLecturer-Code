# Speech module

Folder structure:

- **```pkg/speech2text.py```**: Speech-to-Text functions to transcribe student questions
- **```pkg/text2speech.py```**: Text-to-Speech functions to generate RoboLecturer's voice
- **```pkg/SAD.py```**: Sound Amplitude Detection for background noise level evaluation
- **```test/```**: Testing results and scripts for the speech module
- **```speech.py```**: Main speech script

## Contents

- [Pre-requisites](#pre-requisites)
- [Usage](#usage)
- [Functions](#functions)

## Pre-requisites

Download the required packages:

```
pip install SpeechRecognition pyaudio openai
pip install gTTS TTS pydub
sudo apt-get install ffmpeg espeak
```

## Usage

In order to run the speech module after setup:

```
python3 speech.py
```

## Functions

- **```runS2T(index)```**: microphone index on the system to listen through
- **```runT2S(txt, online=False, speedup=1.2, OUTPUT_PATH="output")```**: text input to generate sound, whether to use online ```googleTTS(txt, OUTPUT_PATH)``` or offline ```DNNTTS(txt, OUTPUT_PATH, GPU=False)``` generating method, playback speed modification and output path of produced audio file
- **```Shush(limit,timel)```**: dB threshold and listening time
