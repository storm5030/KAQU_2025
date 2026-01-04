import os
import json
import re

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

import groq
from dotenv import load_dotenv

import numpy as np
import pandas as pd
import faiss
from sentence_transformers import SentenceTransformer


SYSTEM_PROMPT = """
너는 건물 안내 AI야.
사용자 말에서 목적지(예: 101호, 112호, 로봇 연구실)를 '목록'으로 추출하고,
'갔다가 돌아와/다녀와/복귀/다시 돌아가/원래 자리로' 같은 표현이 있으면 return_to_home=true로 설정해.

반드시 아래 JSON만 출력해. 다른 문장 절대 금지.
{
  "destinations": ["101호", "112호"],
  "return_to_home": true
}
"""

RETURN_PATTERNS = [
    r"갔다가\s*돌아", r"다녀와", r"다녀오", r"복귀", r"돌아와", r"돌아가",
    r"원래\s*자리", r"되돌아", r"다시\s*돌아"
]

def looks_like_return(text: str) -> bool:
    return any(re.search(p, text) for p in RETURN_PATTERNS)


class LLMNode(Node):
    def __init__(self):
        super().__init__('destination_publisher')

        self.plan_pub = self.create_publisher(String, 'destination_plan', 10)
        self.sub = self.create_subscription(String, 'stt_text', self.stt_callback, 10)

        pkg_share_path = get_package_share_directory('kaqu_nav')

        # env
        dotenv_path = os.path.join(pkg_share_path, '.env')
        load_dotenv(dotenv_path=dotenv_path)
        api_key = os.getenv("GROQ_API_KEY")
        if not api_key:
            raise ValueError("GROQ_API_KEY not found in " + dotenv_path)
        self.client = groq.Client(api_key=api_key)

        # map.json
        map_path = os.path.join(pkg_share_path, 'map.json')
        with open(map_path, 'r', encoding='utf-8') as f:
            self.map_data = json.load(f)

        # places -> FAISS 인덱싱 (name 기준)
        self.get_logger().info("FAISS 인덱싱 모델 로드")
        self.embedding_model = SentenceTransformer("all-MiniLM-L6-v2")

        self.df = pd.DataFrame(self.map_data["places"])
        names = self.df["name"].astype(str).tolist()
        embeddings = np.array([self.embedding_model.encode(t) for t in names], dtype="float32")

        self.index = faiss.IndexFlatL2(embeddings.shape[1])
        self.index.add(embeddings)

        self.get_logger().info(f"FAISS 인덱싱 완료. places={len(self.df)}")
        self.get_logger().info("STT 기반 목적지 플랜 퍼블리셔 실행 중")

    def stt_callback(self, msg: String):
        user_input = msg.data.strip()
        if not user_input:
            return
        self.get_logger().info(f"STT 입력: {user_input}")

        plan = self.query_llm_plan(user_input)
        if plan is None:
            # fallback: 3자리 호실만 대충 뽑기
            dests = re.findall(r"\b\d{3}\s*호\b", user_input)
            dests = [d.replace(" ", "") for d in dests]
            plan = {
                "destinations": dests,
                "return_to_home": looks_like_return(user_input)
            }

        destinations = plan.get("destinations", [])
        return_to_home = bool(plan.get("return_to_home", looks_like_return(user_input)))

        if not destinations:
            self.get_logger().error("목적지(destinations)를 찾지 못했습니다.")
            return

        # 문자열 목적지 -> place id 변환 (FAISS)
        dest_ids = []
        for d in destinations:
            pid = self.find_place_id(str(d))
            if pid is None:
                self.get_logger().error(f"'{d}'를 places에서 찾지 못했습니다.")
                return
            dest_ids.append(int(pid))

        out_plan = {
            "dest_ids": dest_ids,
            "return_to_home": return_to_home
        }

        out = String()
        out.data = json.dumps(out_plan, ensure_ascii=False)
        self.plan_pub.publish(out)
        self.get_logger().info(f"destination_plan 퍼블리시: {out.data}")

    def query_llm_plan(self, user_input: str):
        try:
            resp = self.client.chat.completions.create(
                model="llama-3.1-8b-instant",
                messages=[
                    {"role": "system", "content": SYSTEM_PROMPT},
                    {"role": "user", "content": user_input}
                ],
                max_tokens=120,
                temperature=0.0
            )
            text = resp.choices[0].message.content.strip()
            m = re.search(r"\{.*\}", text, flags=re.DOTALL)
            if not m:
                return None
            obj = json.loads(m.group(0))
            if "destinations" not in obj:
                return None
            if "return_to_home" not in obj:
                obj["return_to_home"] = looks_like_return(user_input)
            return obj
        except Exception as e:
            self.get_logger().error(f"LLM 호출/파싱 오류: {e}")
            return None

    def find_place_id(self, query: str):
        q = self.embedding_model.encode(query).reshape(1, -1).astype("float32")
        _, indices = self.index.search(q, k=1)
        idx = int(indices[0][0])
        if idx < 0:
            return None
        row = self.df.iloc[idx]
        return int(row["id"])


def main(args=None):
    rclpy.init(args=args)
    node = LLMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()