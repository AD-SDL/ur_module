"""Base module tests."""

import unittest
from unittest.mock import patch


class TestModule_Base(unittest.TestCase):
    """Base test class for this module."""

    pass


class TestImports(TestModule_Base):
    """Test the imports of the module are working correctly"""

    @patch("sys.argv", ["test", "--ur_ip", "164.54.116.129"])
    def test_driver_import(self):
        """Test the driver and rest node imports"""
        import ur_driver
        import ur_rest_node

        assert ur_driver
        assert ur_rest_node


if __name__ == "__main__":
    unittest.main()
