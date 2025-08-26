from typing import List

def simple_contact_state(
    tau_vec: List[float],
    stance_on: float = 0.15,
    stance_off: float = 0.05,
    overload_on: float = 1.0
) -> List[int]:
    """
    단순 지지 판정 함수.
    - tau_vec: 토크 배열 (예: [fl, fr, rl, rr])
    - stance_on: 지지로 전환하는 임계치
    - stance_off: 스윙으로 전환하는 임계치
    - overload_on: 과하중 상태 전환 임계치 (2로 리턴, 필요 없으면 무시)

    상태 코드:
      0 = SWING
      1 = STANCE
      2 = OVERLOAD (옵션)
    """

    states = []
    for tau in tau_vec:
        if tau >= overload_on:
            states.append(2)  # 과하중 (한 발만 계단 위 등)
        elif tau >= stance_on:
            states.append(1)  # 지지
        elif tau <= stance_off:
            states.append(0)  # 스윙
        else:
            # 중간 구간 → 직전 상태 유지가 이상적이나,
            # 여기서는 단순화를 위해 지지(1)로 취급
            states.append(1)

    return states
