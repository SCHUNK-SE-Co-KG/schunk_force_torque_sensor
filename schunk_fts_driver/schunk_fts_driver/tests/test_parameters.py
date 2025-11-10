# Copyright 2025 SCHUNK SE & Co. KG
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program. If not, see <https://www.gnu.org/licenses/>.
# --------------------------------------------------------------------------------
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters, SetParameters, ListParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


DRIVER_PARAMETERS = [
    "host",
    "port",
    "streaming_port",
]


def test_whether_we_cover_all_driver_parameters(driver):
    node = Node("test_startup_parameters")
    list_params_client = node.create_client(
        ListParameters, "/schunk/driver/list_parameters"
    )
    assert list_params_client.wait_for_service(timeout_sec=2)

    # Meta-check if we test all our node parameters
    future = list_params_client.call_async(ListParameters.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    for param in DRIVER_PARAMETERS:
        assert param in future.result().result.names


def test_driver_has_expected_parameters_after_startup(driver, sensor):
    HOST, PORT = sensor
    node = Node("test_startup_parameters")
    get_params_client = node.create_client(
        GetParameters, "/schunk/driver/get_parameters"
    )
    assert get_params_client.wait_for_service(timeout_sec=2)

    default_parameters = [
        Parameter(
            name="host",
            value=ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value=HOST
            ),
        ),
        Parameter(
            name="port",
            value=ParameterValue(
                type=ParameterType.PARAMETER_INTEGER, integer_value=PORT
            ),
        ),
        Parameter(
            name="streaming_port",
            value=ParameterValue(
                type=ParameterType.PARAMETER_INTEGER, integer_value=54843
            ),
        ),
    ]
    future = get_params_client.call_async(
        GetParameters.Request(names=DRIVER_PARAMETERS)
    )
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    assert len(future.result().values) == len(default_parameters)
    for param, expected in zip(default_parameters, future.result().values):
        assert param.value == expected


def test_driver_supports_setting_parameters(driver):
    node = Node("test_setting_parameters")
    get_params_client = node.create_client(
        GetParameters, "/schunk/driver/get_parameters"
    )
    set_params_client = node.create_client(
        SetParameters, "/schunk/driver/set_parameters"
    )
    assert get_params_client.wait_for_service(timeout_sec=2)
    assert set_params_client.wait_for_service(timeout_sec=2)

    parameters = [
        Parameter(
            name="host",
            value=ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value="1.2.3.4"
            ),
        ),
        Parameter(
            name="port",
            value=ParameterValue(
                type=ParameterType.PARAMETER_INTEGER, integer_value=42
            ),
        ),
        Parameter(
            name="streaming_port",
            value=ParameterValue(
                type=ParameterType.PARAMETER_INTEGER, integer_value=12345
            ),
        ),
    ]

    future = set_params_client.call_async(SetParameters.Request(parameters=parameters))
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    print(future.result())
    future = get_params_client.call_async(
        GetParameters.Request(names=DRIVER_PARAMETERS)
    )
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    assert len(future.result().values) == len(parameters)
    for param, expected in zip(parameters, future.result().values):
        assert param.value == expected


def test_valid_ip_address_formats(driver):
    """Test that various valid IP address formats are accepted."""
    node = Node("test_valid_ip_formats")
    set_params_client = node.create_client(
        SetParameters, "/schunk/driver/set_parameters"
    )
    get_params_client = node.create_client(
        GetParameters, "/schunk/driver/get_parameters"
    )
    assert set_params_client.wait_for_service(timeout_sec=2)
    assert get_params_client.wait_for_service(timeout_sec=2)

    valid_ips = [
        "192.168.0.1",
        "10.0.0.1",
        "127.0.0.1",
        "172.16.0.1",
        "255.255.255.255",
        "0.0.0.0",
    ]

    for ip in valid_ips:
        param = Parameter(
            name="host",
            value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=ip),
        )
        future = set_params_client.call_async(SetParameters.Request(parameters=[param]))
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

        # Verify parameter was set
        get_future = get_params_client.call_async(GetParameters.Request(names=["host"]))
        rclpy.spin_until_future_complete(node, get_future, timeout_sec=10.0)
        result = get_future.result()
        assert result.values[0].string_value == ip, f"Failed to set IP {ip}"

    node.destroy_node()


def test_port_range_validation(driver):
    """Test that port numbers are validated within valid range."""
    node = Node("test_port_range")
    set_params_client = node.create_client(
        SetParameters, "/schunk/driver/set_parameters"
    )
    get_params_client = node.create_client(
        GetParameters, "/schunk/driver/get_parameters"
    )
    assert set_params_client.wait_for_service(timeout_sec=2)
    assert get_params_client.wait_for_service(timeout_sec=2)

    # Test valid port numbers
    valid_ports = [1, 80, 443, 8080, 54843, 65535]

    for port in valid_ports:
        param = Parameter(
            name="port",
            value=ParameterValue(
                type=ParameterType.PARAMETER_INTEGER, integer_value=port
            ),
        )
        future = set_params_client.call_async(SetParameters.Request(parameters=[param]))
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

        # Verify parameter was set
        get_future = get_params_client.call_async(GetParameters.Request(names=["port"]))
        rclpy.spin_until_future_complete(node, get_future, timeout_sec=10.0)
        result = get_future.result()
        assert result.values[0].integer_value == port, f"Failed to set port {port}"

    node.destroy_node()


def test_streaming_port_validation(driver):
    """Test that streaming port parameter accepts valid values."""
    node = Node("test_streaming_port")
    set_params_client = node.create_client(
        SetParameters, "/schunk/driver/set_parameters"
    )
    get_params_client = node.create_client(
        GetParameters, "/schunk/driver/get_parameters"
    )
    assert set_params_client.wait_for_service(timeout_sec=2)
    assert get_params_client.wait_for_service(timeout_sec=2)

    valid_ports = [1024, 8080, 54843, 60000, 65000]

    for port in valid_ports:
        param = Parameter(
            name="streaming_port",
            value=ParameterValue(
                type=ParameterType.PARAMETER_INTEGER, integer_value=port
            ),
        )
        future = set_params_client.call_async(SetParameters.Request(parameters=[param]))
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

        # Verify parameter was set
        get_future = get_params_client.call_async(
            GetParameters.Request(names=["streaming_port"])
        )
        rclpy.spin_until_future_complete(node, get_future, timeout_sec=10.0)
        result = get_future.result()
        assert result.values[0].integer_value == port

    node.destroy_node()


def test_parameter_persistence_across_queries(driver):
    """Test that parameters persist their values across multiple queries."""
    node = Node("test_param_persistence")
    set_params_client = node.create_client(
        SetParameters, "/schunk/driver/set_parameters"
    )
    get_params_client = node.create_client(
        GetParameters, "/schunk/driver/get_parameters"
    )
    assert set_params_client.wait_for_service(timeout_sec=2)
    assert get_params_client.wait_for_service(timeout_sec=2)

    # Set a parameter
    test_ip = "10.20.30.40"
    param = Parameter(
        name="host",
        value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=test_ip),
    )
    future = set_params_client.call_async(SetParameters.Request(parameters=[param]))
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

    # Query multiple times
    for _ in range(3):
        get_future = get_params_client.call_async(GetParameters.Request(names=["host"]))
        rclpy.spin_until_future_complete(node, get_future, timeout_sec=10.0)
        result = get_future.result()
        assert (
            result.values[0].string_value == test_ip
        ), "Parameter should persist across queries"

    node.destroy_node()


def test_multiple_parameter_updates(driver):
    """Test updating multiple parameters in sequence."""
    node = Node("test_multiple_updates")
    set_params_client = node.create_client(
        SetParameters, "/schunk/driver/set_parameters"
    )
    get_params_client = node.create_client(
        GetParameters, "/schunk/driver/get_parameters"
    )
    assert set_params_client.wait_for_service(timeout_sec=2)
    assert get_params_client.wait_for_service(timeout_sec=2)

    # Update 1
    params1 = [
        Parameter(
            name="host",
            value=ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value="192.168.1.1"
            ),
        ),
    ]
    future1 = set_params_client.call_async(SetParameters.Request(parameters=params1))
    rclpy.spin_until_future_complete(node, future1, timeout_sec=10.0)

    # Update 2
    params2 = [
        Parameter(
            name="port",
            value=ParameterValue(
                type=ParameterType.PARAMETER_INTEGER, integer_value=8080
            ),
        ),
    ]
    future2 = set_params_client.call_async(SetParameters.Request(parameters=params2))
    rclpy.spin_until_future_complete(node, future2, timeout_sec=10.0)

    # Verify both are set correctly
    get_future = get_params_client.call_async(
        GetParameters.Request(names=["host", "port"])
    )
    rclpy.spin_until_future_complete(node, get_future, timeout_sec=10.0)
    result = get_future.result()

    assert result.values[0].string_value == "192.168.1.1"
    assert result.values[1].integer_value == 8080

    node.destroy_node()


def test_parameter_type_consistency(driver):
    """Test that parameters maintain their correct types."""
    node = Node("test_type_consistency")
    get_params_client = node.create_client(
        GetParameters, "/schunk/driver/get_parameters"
    )
    assert get_params_client.wait_for_service(timeout_sec=2)

    future = get_params_client.call_async(
        GetParameters.Request(names=DRIVER_PARAMETERS)
    )
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    result = future.result()

    # host should be STRING
    assert result.values[0].type == ParameterType.PARAMETER_STRING
    # port should be INTEGER
    assert result.values[1].type == ParameterType.PARAMETER_INTEGER
    # streaming_port should be INTEGER
    assert result.values[2].type == ParameterType.PARAMETER_INTEGER

    node.destroy_node()


def test_empty_string_host_parameter(driver):
    """Test behavior with empty string for host parameter."""
    node = Node("test_empty_host")
    set_params_client = node.create_client(
        SetParameters, "/schunk/driver/set_parameters"
    )
    assert set_params_client.wait_for_service(timeout_sec=2)

    param = Parameter(
        name="host",
        value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=""),
    )
    future = set_params_client.call_async(SetParameters.Request(parameters=[param]))
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

    # Empty string should be accepted (even if it won't work for connection)
    # The parameter system doesn't validate, that happens at configure time
    assert future.result() is not None

    node.destroy_node()


def test_hostname_as_host_parameter(driver):
    """Test that hostnames (not just IPs) can be set as host parameter."""
    node = Node("test_hostname")
    set_params_client = node.create_client(
        SetParameters, "/schunk/driver/set_parameters"
    )
    get_params_client = node.create_client(
        GetParameters, "/schunk/driver/get_parameters"
    )
    assert set_params_client.wait_for_service(timeout_sec=2)
    assert get_params_client.wait_for_service(timeout_sec=2)

    hostnames = ["localhost", "sensor.local", "my-sensor"]

    for hostname in hostnames:
        param = Parameter(
            name="host",
            value=ParameterValue(
                type=ParameterType.PARAMETER_STRING, string_value=hostname
            ),
        )
        future = set_params_client.call_async(SetParameters.Request(parameters=[param]))
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

        # Verify parameter was set
        get_future = get_params_client.call_async(GetParameters.Request(names=["host"]))
        rclpy.spin_until_future_complete(node, get_future, timeout_sec=10.0)
        result = get_future.result()
        assert result.values[0].string_value == hostname

    node.destroy_node()
