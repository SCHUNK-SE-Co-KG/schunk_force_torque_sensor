from schunk_fts_library.driver import Driver
from schunk_fts_library.utility import Connection


def test_driver_initializes_as_expected():
    driver = Driver()
    assert driver.host is not None
    assert driver.port is not None
    assert isinstance(driver.connection, Connection)
    assert not driver.connection.is_connected
