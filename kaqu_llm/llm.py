import re
import os
import rclpy
import groq
import json
import numpy as np
import faiss
from dotenv import load_dotenv
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sentence_transformers import SentenceTransformer

# 환경 변수 로드
load_dotenv()
GROQ_API_KEY = os.getenv("GROQ_API_KEY")
if not GROQ_API_KEY:
    raise ValueError("API_KEY not found")

# Groq 클라이언트 생성
client = groq.Client(api_key=GROQ_API_KEY)

# FAISS 기반 벡터 검색 모델
embedding_model = SentenceTransformer("all-MiniLM-L6-v2")
json_file_path = "./data/destination.json"
try:
    with open(json_file_path, 'r', encoding='utf-8') as f:
        room_data = json.load(f)
except FileNotFoundError:
    raise FileNotFoundError(f"JSON file not found at {json_file_path}")

# DataFrame으로 변환하여 기존 코드와 호환성 유지
df = pd.DataFrame(room_data)
embeddings = np.array([embedding_model.encode(text) for text in df["Room"].astype(str)], dtype="float32")

# FAISS 인덱스 생성
index = faiss.IndexFlatL2(embeddings.shape[1])
index.add(embeddings)

# 시스템 프롬프트 설정
system_prompt = """
당신은 '창의관' 건물 내부 안내를 담당하는 AI입니다.
사용자가 'OOO으로 가줘', 'OOO로 가줘', 'OOO로 이동해줘' 등과 같은 명령을 내릴 것입니다.
혹은 사용자가 'OOO은 어디야', 'OOO과 가장 가까운 곳은 어디야' 등의 질문을 할 수도 있습니다.
그러면 당신은 목적지를 추출해야 합니다.
"""

class DestinationPublisher(Node):
    def __init__(self):
        super().__init__('destination_publisher')
        self.publisher_ = self.create_publisher(Point, 'destination_coordinates', 10)
        self.json_publisher = self.create_publisher(String, 'destination_json', 10)
        self.subscription = self.create_subscription(String, 'stt_text', self.stt_callback, 10)
        self.get_logger().info("📡 STT 기반 목적지 퍼블리셔 실행 중")

    def stt_callback(self, msg):
        user_input = msg.data.strip()
        self.get_logger().info(f"🎙️ STT 입력: {user_input}")
				
				
        # 이동 명령 (예: ~로 가줘, ~으로 이동해줘)
        move_match = re.match(r"(.+)(로|으로) (가줘|이동해줘)", user_input)
        # 위치 확인 요청 (예: ~는 어디야, ~랑 가장 가까운 곳은 어디야)
        location_match = re.match(r"(.+)(는 어디야|랑 가장 가까운 곳은 어디야)", user_input)

        if move_match:
            destination = move_match.group(1)
            extracted_destination = self.query_llama(user_input)

            if extracted_destination == destination:
                self.get_logger().info(f"🗺️ 목적지 확인됨: {extracted_destination}")
                self.publish_coordinates(extracted_destination)
            else:
                self.get_logger().warn(f"⚠️ AI 응답과 사용자 요청 불일치: ({destination} vs {extracted_destination})")
                self.publish_coordinates(destination)
        elif location_match:
            room = location_match.group(1)
            self.get_logger().info(f"❓ {room}의 위치를 묻는 요청입니다.")
            self.publish_coordinates(room)
        else:
            self.get_logger().warn("❌ 올바른 형식의 이동 명령이 아님.")

    def query_llama(self, user_input):
        response = client.chat.completions.create(
            model="llama3-8b-8192",
            messages=[{"role": "system", "content": system_prompt}, {"role": "user", "content": user_input}],
            max_tokens=64
        )
        response_text = response.choices[0].message.content.strip()
        match_response = re.search(r"알겠습니다. (.+)로 이동하겠습니다.", response_text)
        return match_response.group(1) if match_response else user_input

    def get_coordinates(self, query):
        # FAISS 유사도 검색
        query_embedding = embedding_model.encode(query).reshape(1, -1).astype("float32")
        distances, indices = index.search(query_embedding, k=1)

        if indices[0][0] != -1:
            room_name = df.iloc[indices[0][0]]["Room"]
            x, y, z = df.iloc[indices[0][0]][["X", "Y", "Z"]]
            return room_name, x, y, z
        return None

    def publish_coordinates(self, destination):
        result = self.get_coordinates(destination)
        if result:
            room_name, x, y, z = result
            self.get_logger().info(f"✅ 목적지 {room_name} 좌표: X={x}, Y={y}, Z={z}")

            # Point 메시지 퍼블리시
            point_msg = Point(x=float(x), y=float(y), z=float(z))
            self.publisher_.publish(point_msg)

            # JSON 메시지 퍼블리시
            json_msg = String()
            # x, y, z 값을 float로 변환하여 JSON에 넣기
            json_msg.data = json.dumps({"room": room_name, "x": float(x), "y": float(y), "z": float(z)})
            self.json_publisher.publish(json_msg)

            self.get_logger().info("🚀 좌표가 퍼블리시됨.")
        else:
            self.get_logger().error(f"❌ {destination}의 좌표를 찾을 수 없음.")


def main(args=None):
    rclpy.init(args=args)
    node = DestinationPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
