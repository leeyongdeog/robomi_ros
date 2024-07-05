#!/usr/bin/python3

from gtts import gTTS

tts_text01 = '내용물이 담긴 컵은 반입 금지입니다.'
tts_voice1 = gTTS(text=tts_text01, lang='ko')
tts_voice1.save(r'bbommi_1.mp3')

tts_text02 = '이 전시물은 신라시대에서 전해져온 유물, 찻잔입니다.'
tts_voice2 = gTTS(text=tts_text02, lang='ko')
tts_voice2.save(r'bbommi_2.mp3')

tts_text03 = '전시물 1245번 찻잔이 기울어졌습니다.'
tts_voice3 = gTTS(text=tts_text03, lang='ko')
tts_voice3.save(r'bbommi_3.mp3')

tts_text09 = '삐뽀 삐뽀 로보미, 뽀미 뽀미 로보미 봇입니다.'
tts_voice9 = gTTS(text=tts_text09, lang='ko')
tts_voice9.save(r'bbommi_9.mp3')

