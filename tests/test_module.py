"""Tests the basic functionality of the Module."""

import time
import unittest
from pathlib import Path

import requests
from wei.core.data_classes import ModuleAbout, WorkcellData


class TestWEI_Base(unittest.TestCase):
    """Base class for WEI's pytest tests"""

    def __init__(self, *args, **kwargs):
        """Basic setup for WEI's pytest tests"""
        super().__init__(*args, **kwargs)
        self.root_dir = Path(__file__).resolve().parent.parent
        self.workcell_file = self.root_dir / Path(
            "tests/workcell_defs/test_workcell.yaml"
        )
        self.workcell = WorkcellData.from_yaml(self.workcell_file)
        self.server_host = self.workcell.config.server_host
        self.server_port = self.workcell.config.server_port
        self.url = f"http://{self.server_host}:{self.server_port}"
        self.module_url = "http://ur_module:3011"
        self.redis_host = self.workcell.config.redis_host

        # Check to see that server is up
        start_time = time.time()
        while True:
            try:
                if requests.get(self.url + "/wc/state").status_code == 200:
                    break
            except Exception:
                pass
            time.sleep(1)
            if time.time() - start_time > 60:
                raise TimeoutError("Server did not start in 60 seconds")
        while True:
            try:
                if requests.get(self.module_url + "/state").status_code == 200:
                    break
            except Exception:
                pass
            time.sleep(1)
            if time.time() - start_time > 60:
                raise TimeoutError("Module did not start in 60 seconds")


class TestModuleInterfaces(TestWEI_Base):
    """Tests the basic functionality of the Module."""

    def test_module_about(self):
        """Tests that the module's /about endpoint works"""
        response = requests.get(self.module_url + "/about")
        assert response.status_code == 200
        ModuleAbout(**response.json())


if __name__ == "__main__":
    unittest.main()
