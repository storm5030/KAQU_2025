import math

class LowPassEMA:
    """
    1차 저역통과필터 (Exponential Moving Average, EMA)

    특징
    - 첫 입력을 자동으로 초기 상태로 사용(초기 튐 방지)
    - 기본값: fs=200 Hz, fc=0.5 Hz → alpha 자동 계산
    - 단일 스칼라 입력(roll/pitch/yaw 등) 필터링

    사용 예:
        lpf = LowPassEMA()
        y = lpf.update(x)
    """

    def __init__(self, init=0.0):
        # 고정 기본 샘플링/컷오프 설정
        fs = 200.0   # IMU 발행 주파수 [Hz]
        fc = 0.5     # 컷오프 주파수 [Hz]

        # alpha 계산
        self.alpha = 1.0 - math.exp(-2.0 * math.pi * fc / fs)

        # 내부 상태
        self._y = float(init)
        self._initialized = (init != 0.0)  # init이 0.0이 아니면 초기화된 상태로 간주

    def update(self, x: float) -> float:
        """
        새 입력 x를 받아 필터 출력 반환.
        첫 호출 시엔 입력을 그대로 출력(초기 튐 방지).
        """
        x = float(x)

        if not self._initialized:
            self._y = x
            self._initialized = True
            return self._y

        self._y = (1.0 - self.alpha) * self._y + self.alpha * x
        return self._y

    def reset(self, value=0.0) -> None:
        """
        필터 상태 초기화.
        - value가 0.0이면 '미초기화 상태'로 돌아감
        - 그 외 값이면 해당 값으로 바로 초기화
        """
        self._y = float(value)
        self._initialized = (value != 0.0)

    @property
    def y(self) -> float:
        """현재 필터 출력값 조회"""
        return self._y
