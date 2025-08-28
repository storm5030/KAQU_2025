#찐 최종_찐막
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import pyaudio # Python에서 마이크, 스피커 같은 오디오 장치에 접근할 수 있게 해주는 라이브러리
import numpy as np
import time
import noisereduce as nr

class WhisperSTTNode(Node):
    def __init__(self):
        super().__init__('whisper_stt_node')
        self.get_logger().info("Whisper 모델 로드 중...")
        self.model = whisper.load_model("small") 
        
        self.publisher_ = self.create_publisher(String, 'stt_text', 10)
        
        # 오디오 설정
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000        # Whisper 권장 샘플링 레이트
        self.CHUNK = 1024        # 버퍼 크기
        self.RECORD_SECONDS = 60  # 60초 단위로 오디오를 처리
        
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(format=self.FORMAT,
                                      channels=self.CHANNELS,
                                      rate=self.RATE,
                                      input=True,
                                      frames_per_buffer=self.CHUNK)
        self.get_logger().info("🎤 실시간 한국어 음성 인식 시작!")
        self.get_logger().info("시스템 활성화를 위해 '안녕'이라고 말해주세요.")
		
		#오디오 녹음 함수 
    def record_audio_segment(self, seconds):
        frames = []
        num_chunks = int(self.RATE / self.CHUNK * seconds)
        for _ in range(num_chunks):
            data = self.stream.read(self.CHUNK, exception_on_overflow=False)
            frames.append(data)
        # 녹음된 데이터: int16 -> float32, [-1, 1] 범위로 정규화
        audio_data = np.frombuffer(b''.join(frames), dtype=np.int16).astype(np.float32) / 32768.0

        # 오디오 전처리: 노이즈 제거
        noise_duration = 0.2  # 녹음된 첫 0.2초를 노이즈 샘플로 사용
        noise_samples = int(self.RATE * noise_duration)
        if len(audio_data) > noise_samples:
            noise_clip = audio_data[:noise_samples]
            # noisereduce 라이브러리의 올바른 인자명을 사용 (y, sr, y_noise)
            audio_data = nr.reduce_noise(y=audio_data, sr=self.RATE, y_noise=noise_clip)
        
        # 전처리 후 다시 정규화 (신호의 최대값을 이용)
        max_val = np.max(np.abs(audio_data))
        if max_val > 0:
            audio_data = audio_data / max_val
        
        return audio_data

    def publish_text(self, text):
        msg = String()
        msg.data = text
        self.publisher_.publish(msg)
        self.get_logger().info(f"퍼블리시된 텍스트: {text}")
		
		
    def run(self):
        activated = False
        while rclpy.ok():
            if not activated:
                # 활성화 전: 5초 녹음 후 "안녕" 포함 여부 확인
                audio_data = self.record_audio_segment(self.RECORD_SECONDS)
                result = self.model.transcribe(audio_data, fp16=False, language="ko", task="transcribe", temperature=0)
                recognized_text = result["text"].strip()
                if "안녕" in recognized_text:
                    activated = True
                    self.get_logger().info("✅ 시스템 활성화됨. 이제 200초간 명령어를 수신합니다.")
                    self.publish_text("시스템 활성화됨")
                else:
                    self.get_logger().info("초기 활성화를 위해 '안녕'을 말해주세요.")
            else:
                # 활성화 후: 200초 동안 명령어 수신
                start_time = time.time()
                while time.time() - start_time < 200:
                    audio_data = self.record_audio_segment(self.RECORD_SECONDS)
                    result = self.model.transcribe(audio_data, fp16=False, language="ko", task="transcribe", temperature=0)
                    recognized_text = result["text"].strip()
                    if recognized_text:
                        self.get_logger().info(f"명령어 인식: {recognized_text}")
                        self.publish_text(recognized_text)
                activated = False
                self.get_logger().info("⌛ 명령어 수신 시간이 종료되었습니다. 다시 '안녕'으로 활성화하세요.")

    def stop(self):
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()

def main(args=None):
    rclpy.init(args=args)
    node = WhisperSTTNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("🛑 음성 인식 종료!")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

