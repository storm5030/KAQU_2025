// fileName: llm_py_faiss_revised.py
import re
import os
import rclpy
import groq
import json
import pandas as pd
import numpy as np
import faiss
from dotenv import load_dotenv
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sentence_transformers import SentenceTransformer

load_dotenv()
GROQ_API_KEY = os.getenv("GROQ_API_KEY")
if not GROQ_API_KEY:
    raise ValueError("API_KEY not found")

client = groq.Client(api_key=GROQ_API_KEY)

system_prompt = """
당신은 '창의관' 건물 내부 안내를 담당하는 AI입니다.
사용자가 'OOO으로 가줘', 'OOO로 가줘', 'OOO로 이동해줘' 등과 같은 명령을 내릴 것입니다.
혹은 사용자가 'OOO은 어디야', 'OOO과 가장 가까운 곳은 어디야' 등의 질문을 할 수도 있습니다.
사용자의 말에서 '112호', '메이커스 스페이스', '로봇 연구실' 처럼 목적지를 나타내는 핵심 키워드만 간결하게 추출해야 합니다.
"""

class DestinationPublisher(Node):
    def __init__(self):
        super().__init__('destination_publisher')
        self.publisher_ = self.create_publisher(Point, 'destination_coordinates', 10)
        self.subscription = self.create_subscription(String, 'stt_text', self.stt_callback, 10)
        
        try:
            with open('map.json', 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception as e:
            self.get_logger().error(f"지도 파일 로딩 중 오류 발생: {e}")
            self.destroy_node()
            return None

        try:
            self.get_logger().info("FAISS 인덱싱 모델 로드")
            self.embedding_model = SentenceTransformer("all-MiniLM-L6-v2")
            
            self.df = pd.DataFrame(self.map_data['nodes'])
            
            embeddings = np.array(
                [self.embedding_model.encode(text) for text in self.df["name"].astype(str)], 
                dtype="float32"
            )

            self.index = faiss.IndexFlatL2(embeddings.shape[1])
            self.index.add(embeddings)
            self.get_logger().info("FAISS 인덱싱 완료.")

        except Exception as e:
            self.get_logger().error(f"FAISS 모델 오류 발생: {e}")
            self.destroy_node()
            return
            
        self.get_logger().info("STT 기반 목적지 퍼블리셔 실행 중")

    def stt_callback(self, msg):
        user_input = msg.data.strip()
        self.get_logger().info(f"STT 입력: {user_input}")
        
        # LLM으로 핵심 목적지 키워드 추출
        destination_keyword = self.query_llama(user_input)
        if not destination_keyword:
            self.get_logger().warn("LLM으로부터 목적지를 추출하지 못했습니다.")
            return

        self.get_logger().info(f"추출된 목적지: {destination_keyword}")
        
        # FAISS로 가장 유사한 목적지 검색
        self.find_destination(destination_keyword)

    def query_llama(self, user_input):
        try:
            response = client.chat.completions.create(
                model="llama3-8b-8192",
                messages=[{"role": "system", "content": system_prompt}, {"role": "user", "content": user_input}],
                max_tokens=32,
                temperature=0.0
            )
            return response.choices[0].message.content.strip().replace("'", "").replace('"', '')
        except Exception as e:
            self.get_logger().error(f"Groq API 호출 중 오류 발생: {e}")
            return None

    def find_destination(self, query):
        # 벡터로 변환
        query_embedding = self.embedding_model.encode(query).reshape(1, -1).astype("float32")
        
        # 가장 유사한 벡터 검색 (가장 가까운 1개를 찾도록 설정)
        distances, indices = self.index.search(query_embedding, k=1)

        if indices[0][0] != -1:
            result = self.df.iloc[indices[0][0]]
            
            node_id = int(result["id"])
            name = result["name"]
            x, y, z = float(result["x"]), float(result["y"]), float(result["z"])

            # Point 메시지 퍼블리시
            point_msg = Point(x=x, y=y, z=z)
            self.publisher_.publish(point_msg)
        else:
            self.get_logger().error(f"'{query}'와 유사한 목적지를 찾을 수 없음.")


def main(args=None):
    rclpy.init(args=args)
    node = DestinationPublisher()
    if not node.is_destroyed():
        rclpy.spin(node)
        node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()