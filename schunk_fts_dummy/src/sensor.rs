use bytes::{BufMut, BytesMut};
use tokio::io::{AsyncRead, AsyncWrite};

pub struct Sensor<T> {
    stream: T,
    valid_commands: Vec<u8>,
}

impl<T> Sensor<T>
where
    T: AsyncRead + AsyncWrite + Unpin + Send + 'static,
{
    pub fn new(stream: T) -> Self {
        Self {
            stream,
            valid_commands: vec![0x10, 0x11, 0x12, 0x13, 0x20, 0x30, 0x31, 0x40, 0x41],
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
            let error_code = 0x00;
            let param_index = u16::from_le_bytes([msg[7], msg[8]]);
            let param_subindex = msg[9];
            let param_value = "KMS".as_bytes();
            let mut response = BytesMut::with_capacity(6);
            response.put_u8(command_id);
            response.put_u8(error_code);
            response.put_u16_le(param_index);
            response.put_u8(param_subindex);
            response.put_slice(param_value);
            println!("Get parameter: {param_index:x}, {param_subindex:x}");
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
}
