from gtts import gTTS
import pyglet
from playsound import playsound



tts=gTTS(text='กดลิฟต์ไปชั้น6',lang='th')
tts1=gTTS(text='กำลังเคลื่อนที่เข้าข้างในลิฟต์',lang='th')
tts2=gTTS(text='กรุณาออกห่างจากหุ่นยนต์',lang='th')
tts3=gTTS(text='กำลังไปชั้น6',lang='th')
tts4=gTTS(text='ขอทางออกจากลิฟต์ด้วยค่ะ',lang='th')
tts5=gTTS(text='ถึงชั้น6แล้ว',lang='th')


tts.save('draft.mp3')
tts1.save('draft1.mp3')
tts2.save('draft2.mp3')
tts3.save('draft3.mp3')
tts4.save('draft4.mp3')
tts5.save('draft5.mp3')

# playsound("/home/earththesis/ros2_directory/3wheel_ws/draft.mp3")
# playsound("/home/earththesis/ros2_directory/3wheel_ws/draft1.mp3")
# playsound("/home/earththesis/ros2_directory/3wheel_ws/draft2.mp3")
# playsound("/home/earththesis/ros2_directory/3wheel_ws/draft3.mp3")
# playsound("/home/earththesis/ros2_directory/3wheel_ws/draft4.mp3")

# Playing the converted file
# sound = pyglet.media.load("/home/earththesis/ros2_directory/3wheel_ws/output.mp3")

# sound.play()