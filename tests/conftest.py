"""
pytest 公共 fixture

提供共享的 QApplication 实例，避免每个测试模块各自创建。
"""
import pytest


@pytest.fixture(scope="session")
def qapp():
    """返回单例 QApplication（整个测试会话只创建一次）"""
    pytest.importorskip("PyQt5")
    from PyQt5.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    yield app
