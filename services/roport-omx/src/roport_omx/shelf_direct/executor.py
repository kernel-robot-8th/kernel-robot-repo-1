"""DirectExecutor: 현장 PanelExecutor 그대로, Panel HTTP 호출만 같은 PC 안의 LocalPanel로 돌린다.

완료 근거·정지 확인·HOLD·복구 판정은 panel_executor.py 코드를 그대로 쓴다(두 벌로 갈라지지 않게).
"""

from __future__ import annotations

from ..panel_executor import PanelExecutor
from .local_panel import LocalPanel


class DirectExecutor(PanelExecutor):
    def __init__(self, routes, local: LocalPanel, **kwargs):
        super().__init__(routes, base_url="local://shelf-direct", **kwargs)
        self.local = local

    def _call(self, method: str, path: str, body: dict | None = None, timeout=None):
        return self.local.request(method, path, body)

    def close(self) -> None:
        self.local.close()
