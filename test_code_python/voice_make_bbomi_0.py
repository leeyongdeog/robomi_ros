#!/usr/bin/python3

from gtts import gTTS

tts_text01 = '뽀미 뽀미 뽀미 뽀미 순찰 로오오봇 로보미입니다. 뽀미 뽀미 단속 로오오봇 로보미입니다. 뽀미 뽀미 안내 로오오봇 로보미입니다.'
tts_voice1 = gTTS(text=tts_text01, lang='ko')
tts_voice1.save(r'bbommi.mp3')
