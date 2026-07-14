use crate::output_rate::OutputRateState;
use crate::udp_destination_port::UdpDestinationPortState;
use bytes::{BufMut, BytesMut};
use tokio::io::{AsyncRead, AsyncWrite};

pub struct Sensor<T> {
    stream: T,
    valid_commands: Vec<u8>,
    output_rate: OutputRateState,
    udp_destination_port: UdpDestinationPortState,
}

impl<T> Sensor<T>
where
    T: AsyncRead + AsyncWrite + Unpin + Send + 'static,
{
    #[cfg(test)]
    pub fn new(stream: T) -> Self {
        Self::with_state(
            stream,
            OutputRateState::default(),
            UdpDestinationPortState::default(),
        )
    }

    pub fn with_state(
        stream: T,
        output_rate: OutputRateState,
        udp_destination_port: UdpDestinationPortState,
    ) -> Self {
        Self {
            stream,
            valid_commands: vec![0x10, 0x11, 0x12, 0x13, 0x20, 0x30, 0x31, 0x40, 0x41],
            output_rate,
            udp_destination_port,
        }
    }

    pub async fn read(&mut self) -> tokio::io::Result<Vec<u8>> {
        use tokio::io::AsyncReadExt;
        let mut buf = [0; 1024];
        let n = self.stream.read(&mut buf).await?;
        Ok(buf[..n].to_vec())
    }

    pub async fn write(&mut self, msg: &[u8]) -> tokio::io::Result<()> {
        use tokio::io::AsyncWriteExt;
        self.stream.write_all(msg).await
    }

    pub async fn process(&mut self, msg: &[u8]) -> tokio::io::Result<BytesMut> {
        let command_id = msg[6];

        // Getting parameters
        if command_id == 0xf0 {
            let param_index = u16::from_le_bytes([msg[7], msg[8]]);
            let param_subindex = msg[9];
            let output_rate_value = [self.output_rate.get().enum_value];
            let udp_destination_port_value = self.udp_destination_port.get().to_le_bytes();
            let (error_code, param_value): (u8, &[u8]) = match (param_index, param_subindex) {
                (0x0001, 0x00 | 0x03) => (0x00, "KMS".as_bytes()),
                (0x1020, 0x00) => (0x00, &output_rate_value),
                (0x1033, 0x00) => (0x00, &udp_destination_port_value),
                (0x0001 | 0x1020 | 0x1033, _) => (0x14, &[]),
                _ => (0x13, &[]),
            };
            let mut response = BytesMut::with_capacity(6);
            response.put_u8(command_id);
            response.put_u8(error_code);
            response.put_u16_le(param_index);
            response.put_u8(param_subindex);
            response.put_slice(param_value);
            println!("Get parameter: {param_index:x}, {param_subindex:x}");
            return Ok(response);
        }

        // Setting parameters
        if command_id == 0xf1 {
            let param_index = u16::from_le_bytes([msg[7], msg[8]]);
            let param_subindex = msg[9];
            let error_code = if param_index == 0x1020 && param_subindex == 0x00 {
                if msg.len() <= 10 {
                    0x15
                } else if self.output_rate.set_enum(msg[10]) {
                    0x00
                } else {
                    0x16
                }
            } else if param_index == 0x1033 && param_subindex == 0x00 {
                if msg.len() < 12 {
                    0x15
                } else {
                    let port = u16::from_le_bytes([msg[10], msg[11]]);
                    if self.udp_destination_port.set(port) {
                        0x00
                    } else {
                        0x16
                    }
                }
            } else {
                0x00
            };
            let mut response = BytesMut::with_capacity(6);
            response.put_u8(command_id);
            response.put_u8(error_code);
            response.put_u16_le(param_index);
            response.put_u8(param_subindex);
            println!("Set parameter: {param_index:x}, {param_subindex:x}");
            return Ok(response);
        }

        // General commands
        if self.valid_commands.contains(&command_id) {
            let error_code = 0x00;
            let mut response = BytesMut::with_capacity(2);
            response.put_u8(command_id);
            response.put_u8(error_code);
            println!("Run command: {command_id:x}");
            return Ok(response);
        } else {
            let error_code = 0x01;
            let mut response = BytesMut::with_capacity(2);
            response.put_u8(command_id);
            response.put_u8(error_code);
            println!("Invalid command: {command_id:x}");
            return Ok(response);
        }
    }
}

// Tests -----------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use bytes::{BufMut, BytesMut};
    use tokio::io::{AsyncReadExt, AsyncWriteExt, duplex};

    #[tokio::test]
    async fn test_sensor_offers_reading_data() {
        let (mut client, server) = duplex(1024);
        let mut sensor = Sensor::new(server);

        tokio::spawn(async move {
            let _ = client.write_all(b"test message").await;
        });

        let msg = sensor.read().await.unwrap();
        assert_eq!(msg, b"test message");
    }

    #[tokio::test]
    async fn test_sensor_offers_writing_data() {
        let (client, mut server) = duplex(1024);
        let mut sensor = Sensor::new(client);

        sensor.write(b"Hello back").await.unwrap();

        let mut buf = [0; 1024];
        let n = server.read(&mut buf).await.unwrap();

        let received = &buf[..n];
        assert_eq!(received, b"Hello back");
    }

    #[tokio::test]
    async fn test_sensor_knows_valid_command_ids() {
        let (_, server) = duplex(1024);
        let sensor = Sensor::new(server);
        let valid_commands = [0x10, 0x11, 0x12, 0x13, 0x20, 0x30, 0x31, 0x40, 0x41];
        assert_eq!(sensor.valid_commands, valid_commands);
    }

    #[tokio::test]
    async fn test_sensor_processes_commands() {
        let (mut client, server) = duplex(1024);
        let mut sensor = Sensor::new(server);

        // Known command IDs
        let valid_commands = sensor.valid_commands.clone();
        for cmd in valid_commands {
            let mut command_msg = BytesMut::with_capacity(6);
            command_msg.put_bytes(0xff, 2); // sync bytes
            command_msg.put_u16_le(0x0001); // counter
            command_msg.put_u16_le(0x0001); // payload size
            command_msg.put_u8(cmd);
            client.write_all(&command_msg).await.unwrap();

            let bytes = sensor.read().await.unwrap();
            let response = sensor.process(&bytes).await;
            assert!(response.is_ok());
            let response = response.unwrap().to_vec();
            assert_eq!(response[0], cmd);
            assert_eq!(response[1], 0x00);
        }

        // Unknown command IDS
        let invalid_commands = [0x01, 0x00, 0x42, 0xff];
        for cmd in invalid_commands {
            let mut command_msg = BytesMut::with_capacity(6);
            command_msg.put_bytes(0xff, 2);
            command_msg.put_u16_le(0x0002);
            command_msg.put_u16_le(0x0001);
            command_msg.put_u8(cmd);
            client.write_all(&command_msg).await.unwrap();

            let bytes = sensor.read().await.unwrap();
            let response = sensor.process(&bytes).await;
            assert!(response.is_ok());
            let response = response.unwrap().to_vec();
            assert_eq!(response[0], cmd);
            assert_eq!(response[1], 0x01); // unknown command
        }
    }

    #[tokio::test]
    async fn test_sensor_handles_get_parameter_requests() {
        let (mut client, server) = duplex(1024);
        let mut sensor = Sensor::new(server);

        let param_cmd = 0xf0;
        let param_index = 0x0001;
        let param_subindex = 0x00;

        let mut msg = BytesMut::with_capacity(6);
        msg.put_bytes(0xff, 2);
        msg.put_u16_le(0x0001);
        msg.put_u16_le(0x0001);
        msg.put_u8(param_cmd);
        msg.put_u16_le(param_index);
        msg.put_u8(param_subindex);
        client.write_all(&msg).await.unwrap();

        let bytes = sensor.read().await.unwrap();
        let response = sensor.process(&bytes).await;
        assert!(response.is_ok());
        let response = response.unwrap().to_vec();

        let param_value = "KMS".as_bytes();
        assert_eq!(response[0], param_cmd);
        assert_eq!(response[1], 0x00);
        assert_eq!(u16::from_le_bytes([response[2], response[3]]), param_index);
        assert_eq!(response[4], param_subindex);
        assert_eq!(&response.as_slice()[5..], param_value);
    }

    #[tokio::test]
    async fn test_sensor_rejects_unknown_parameter_reads() {
        let (mut client, server) = duplex(1024);
        let mut sensor = Sensor::new(server);

        let mut msg = BytesMut::with_capacity(10);
        msg.put_bytes(0xff, 2);
        msg.put_u16_le(0x0001);
        msg.put_u16_le(0x0004);
        msg.put_u8(0xf0);
        msg.put_u16_le(0xffff);
        msg.put_u8(0x00);
        client.write_all(&msg).await.unwrap();

        let bytes = sensor.read().await.unwrap();
        let response = sensor.process(&bytes).await.unwrap().to_vec();

        assert_eq!(response[0], 0xf0);
        assert_eq!(response[1], 0x13);
        assert_eq!(u16::from_le_bytes([response[2], response[3]]), 0xffff);
        assert_eq!(response[4], 0x00);
        assert_eq!(response.len(), 5);
    }

    #[tokio::test]
    async fn test_sensor_rejects_unknown_parameter_subindices() {
        let (mut client, server) = duplex(1024);
        let mut sensor = Sensor::new(server);

        let mut msg = BytesMut::with_capacity(10);
        msg.put_bytes(0xff, 2);
        msg.put_u16_le(0x0001);
        msg.put_u16_le(0x0004);
        msg.put_u8(0xf0);
        msg.put_u16_le(0x1020);
        msg.put_u8(0x01);
        client.write_all(&msg).await.unwrap();

        let bytes = sensor.read().await.unwrap();
        let response = sensor.process(&bytes).await.unwrap().to_vec();

        assert_eq!(response[0], 0xf0);
        assert_eq!(response[1], 0x14);
        assert_eq!(u16::from_le_bytes([response[2], response[3]]), 0x1020);
        assert_eq!(response[4], 0x01);
        assert_eq!(response.len(), 5);
    }

    #[tokio::test]
    async fn test_sensor_gets_default_udp_output_rate_parameter() {
        let (mut client, server) = duplex(1024);
        let mut sensor = Sensor::new(server);

        let param_cmd = 0xf0;
        let param_index = 0x1020;
        let param_subindex = 0x00;

        let mut msg = BytesMut::with_capacity(10);
        msg.put_bytes(0xff, 2);
        msg.put_u16_le(0x0001);
        msg.put_u16_le(0x0004);
        msg.put_u8(param_cmd);
        msg.put_u16_le(param_index);
        msg.put_u8(param_subindex);
        client.write_all(&msg).await.unwrap();

        let bytes = sensor.read().await.unwrap();
        let response = sensor.process(&bytes).await.unwrap().to_vec();

        assert_eq!(response[0], param_cmd);
        assert_eq!(response[1], 0x00);
        assert_eq!(u16::from_le_bytes([response[2], response[3]]), param_index);
        assert_eq!(response[4], param_subindex);
        assert_eq!(response[5], 0x00);
    }

    #[tokio::test]
    async fn test_sensor_sets_udp_output_rate_parameter() {
        let (mut client, server) = duplex(1024);
        let mut sensor = Sensor::new(server);

        let param_index = 0x1020;
        let param_subindex = 0x00;
        let requested_rate = 0x0a;

        let mut set_msg = BytesMut::with_capacity(11);
        set_msg.put_bytes(0xff, 2);
        set_msg.put_u16_le(0x0001);
        set_msg.put_u16_le(0x0005);
        set_msg.put_u8(0xf1);
        set_msg.put_u16_le(param_index);
        set_msg.put_u8(param_subindex);
        set_msg.put_u8(requested_rate);
        client.write_all(&set_msg).await.unwrap();

        let bytes = sensor.read().await.unwrap();
        let response = sensor.process(&bytes).await.unwrap().to_vec();

        assert_eq!(response[0], 0xf1);
        assert_eq!(response[1], 0x00);
        assert_eq!(u16::from_le_bytes([response[2], response[3]]), param_index);
        assert_eq!(response[4], param_subindex);

        let mut get_msg = BytesMut::with_capacity(10);
        get_msg.put_bytes(0xff, 2);
        get_msg.put_u16_le(0x0002);
        get_msg.put_u16_le(0x0004);
        get_msg.put_u8(0xf0);
        get_msg.put_u16_le(param_index);
        get_msg.put_u8(param_subindex);
        client.write_all(&get_msg).await.unwrap();

        let bytes = sensor.read().await.unwrap();
        let response = sensor.process(&bytes).await.unwrap().to_vec();

        assert_eq!(response[0], 0xf0);
        assert_eq!(response[1], 0x00);
        assert_eq!(response[5], requested_rate);
    }

    #[tokio::test]
    async fn test_sensor_rejects_invalid_udp_output_rate_parameter() {
        let (mut client, server) = duplex(1024);
        let mut sensor = Sensor::new(server);

        let param_index = 0x1020;
        let param_subindex = 0x00;

        let mut valid_msg = BytesMut::with_capacity(11);
        valid_msg.put_bytes(0xff, 2);
        valid_msg.put_u16_le(0x0001);
        valid_msg.put_u16_le(0x0005);
        valid_msg.put_u8(0xf1);
        valid_msg.put_u16_le(param_index);
        valid_msg.put_u8(param_subindex);
        valid_msg.put_u8(0x01);
        client.write_all(&valid_msg).await.unwrap();
        let bytes = sensor.read().await.unwrap();
        let response = sensor.process(&bytes).await.unwrap().to_vec();
        assert_eq!(response[1], 0x00);

        let mut invalid_msg = BytesMut::with_capacity(11);
        invalid_msg.put_bytes(0xff, 2);
        invalid_msg.put_u16_le(0x0002);
        invalid_msg.put_u16_le(0x0005);
        invalid_msg.put_u8(0xf1);
        invalid_msg.put_u16_le(param_index);
        invalid_msg.put_u8(param_subindex);
        invalid_msg.put_u8(0x04);
        client.write_all(&invalid_msg).await.unwrap();
        let bytes = sensor.read().await.unwrap();
        let response = sensor.process(&bytes).await.unwrap().to_vec();

        assert_eq!(response[0], 0xf1);
        assert_eq!(response[1], 0x16);

        let mut get_msg = BytesMut::with_capacity(10);
        get_msg.put_bytes(0xff, 2);
        get_msg.put_u16_le(0x0003);
        get_msg.put_u16_le(0x0004);
        get_msg.put_u8(0xf0);
        get_msg.put_u16_le(param_index);
        get_msg.put_u8(param_subindex);
        client.write_all(&get_msg).await.unwrap();
        let bytes = sensor.read().await.unwrap();
        let response = sensor.process(&bytes).await.unwrap().to_vec();

        assert_eq!(response[1], 0x00);
        assert_eq!(response[5], 0x01);
    }

    #[tokio::test]
    async fn test_sensor_gets_default_udp_destination_port_parameter() {
        let (mut client, server) = duplex(1024);
        let mut sensor = Sensor::new(server);

        let param_index = 0x1033;
        let param_subindex = 0x00;

        let mut msg = BytesMut::with_capacity(10);
        msg.put_bytes(0xff, 2);
        msg.put_u16_le(0x0001);
        msg.put_u16_le(0x0004);
        msg.put_u8(0xf0);
        msg.put_u16_le(param_index);
        msg.put_u8(param_subindex);
        client.write_all(&msg).await.unwrap();

        let bytes = sensor.read().await.unwrap();
        let response = sensor.process(&bytes).await.unwrap().to_vec();

        assert_eq!(response[0], 0xf0);
        assert_eq!(response[1], 0x00);
        assert_eq!(u16::from_le_bytes([response[2], response[3]]), param_index);
        assert_eq!(response[4], param_subindex);
        assert_eq!(u16::from_le_bytes([response[5], response[6]]), 54843);
    }

    #[tokio::test]
    async fn test_sensor_sets_udp_destination_port_parameter() {
        let (mut client, server) = duplex(1024);
        let port_state = UdpDestinationPortState::default();
        let mut sensor = Sensor::with_state(server, OutputRateState::default(), port_state.clone());

        let param_index = 0x1033;
        let requested_port = 60000;

        let mut set_msg = BytesMut::with_capacity(12);
        set_msg.put_bytes(0xff, 2);
        set_msg.put_u16_le(0x0001);
        set_msg.put_u16_le(0x0006);
        set_msg.put_u8(0xf1);
        set_msg.put_u16_le(param_index);
        set_msg.put_u8(0x00);
        set_msg.put_u16_le(requested_port);
        client.write_all(&set_msg).await.unwrap();

        let bytes = sensor.read().await.unwrap();
        let response = sensor.process(&bytes).await.unwrap().to_vec();

        assert_eq!(response[0], 0xf1);
        assert_eq!(response[1], 0x00);
        assert_eq!(port_state.get(), requested_port);

        let mut get_msg = BytesMut::with_capacity(10);
        get_msg.put_bytes(0xff, 2);
        get_msg.put_u16_le(0x0002);
        get_msg.put_u16_le(0x0004);
        get_msg.put_u8(0xf0);
        get_msg.put_u16_le(param_index);
        get_msg.put_u8(0x00);
        client.write_all(&get_msg).await.unwrap();

        let bytes = sensor.read().await.unwrap();
        let response = sensor.process(&bytes).await.unwrap().to_vec();

        assert_eq!(response[0], 0xf0);
        assert_eq!(response[1], 0x00);
        assert_eq!(
            u16::from_le_bytes([response[5], response[6]]),
            requested_port
        );
    }

    #[tokio::test]
    async fn test_sensor_rejects_invalid_udp_destination_port_parameter() {
        let (mut client, server) = duplex(1024);
        let port_state = UdpDestinationPortState::default();
        let mut sensor = Sensor::with_state(server, OutputRateState::default(), port_state.clone());

        let param_index = 0x1033;

        let mut valid_msg = BytesMut::with_capacity(12);
        valid_msg.put_bytes(0xff, 2);
        valid_msg.put_u16_le(0x0001);
        valid_msg.put_u16_le(0x0006);
        valid_msg.put_u8(0xf1);
        valid_msg.put_u16_le(param_index);
        valid_msg.put_u8(0x00);
        valid_msg.put_u16_le(60000);
        client.write_all(&valid_msg).await.unwrap();
        let bytes = sensor.read().await.unwrap();
        let response = sensor.process(&bytes).await.unwrap().to_vec();
        assert_eq!(response[1], 0x00);

        let mut too_short_msg = BytesMut::with_capacity(11);
        too_short_msg.put_bytes(0xff, 2);
        too_short_msg.put_u16_le(0x0002);
        too_short_msg.put_u16_le(0x0005);
        too_short_msg.put_u8(0xf1);
        too_short_msg.put_u16_le(param_index);
        too_short_msg.put_u8(0x00);
        too_short_msg.put_u8(0x01);
        client.write_all(&too_short_msg).await.unwrap();
        let bytes = sensor.read().await.unwrap();
        let response = sensor.process(&bytes).await.unwrap().to_vec();
        assert_eq!(response[1], 0x15);
        assert_eq!(port_state.get(), 60000);

        for (counter, invalid_port) in [(3, 0), (4, 65535)] {
            let mut invalid_msg = BytesMut::with_capacity(12);
            invalid_msg.put_bytes(0xff, 2);
            invalid_msg.put_u16_le(counter);
            invalid_msg.put_u16_le(0x0006);
            invalid_msg.put_u8(0xf1);
            invalid_msg.put_u16_le(param_index);
            invalid_msg.put_u8(0x00);
            invalid_msg.put_u16_le(invalid_port);
            client.write_all(&invalid_msg).await.unwrap();
            let bytes = sensor.read().await.unwrap();
            let response = sensor.process(&bytes).await.unwrap().to_vec();
            assert_eq!(response[1], 0x16);
            assert_eq!(port_state.get(), 60000);
        }
    }

    #[tokio::test]
    async fn test_sensor_handles_set_parameter_requests() {
        let (mut client, server) = duplex(1024);
        let mut sensor = Sensor::new(server);

        let param_cmd = 0xf1;
        let param_index = 0x0001;
        let param_subindex = 0x00;

        let mut msg = BytesMut::with_capacity(6);
        msg.put_bytes(0xff, 2);
        msg.put_u16_le(0x0001);
        msg.put_u16_le(0x0001);
        msg.put_u8(param_cmd);
        msg.put_u16_le(param_index);
        msg.put_u8(param_subindex);
        msg.put_slice("some-arbitrary-value".as_bytes());
        client.write_all(&msg).await.unwrap();

        let bytes = sensor.read().await.unwrap();
        let response = sensor.process(&bytes).await;
        assert!(response.is_ok());
        let response = response.unwrap().to_vec();

        assert_eq!(response[0], param_cmd);
        assert_eq!(response[1], 0x00);
        assert_eq!(u16::from_le_bytes([response[2], response[3]]), param_index);
        assert_eq!(response[4], param_subindex);
    }
}
