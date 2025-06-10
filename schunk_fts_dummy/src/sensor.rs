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
            valid_commands: vec![
                0x10, 0x11, 0x12, 0x13, 0x20, 0x30, 0x31, 0x40, 0x41, 0xf0, 0xf1,
            ],
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

    pub async fn process(&mut self, msg: &[u8]) -> tokio::io::Result<i8> {
        let mut response = 0x00;
        let command_idx = 6;
        if !self.valid_commands.contains(&msg[command_idx]) {
            response = 0x01;
        }
        Ok(response)
    }
}

// Tests -----------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
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
        let valid_commands = [
            0x10, 0x11, 0x12, 0x13, 0x20, 0x30, 0x31, 0x40, 0x41, 0xf0, 0xf1,
        ];
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
            command_msg.put_u16(0x0001); // counter
            command_msg.put_u16(0x0001); // payload size
            command_msg.put_u8(cmd);
            client.write_all(&command_msg).await.unwrap();

            let bytes = sensor.read().await.unwrap();
            let response = sensor.process(&bytes).await;
            assert!(response.is_ok());
            assert_eq!(response.unwrap(), 0x00)
        }

        // Unknown command IDS
        let invalid_commands = [0x01, 0x00, 0x42, 0xff];
        for cmd in invalid_commands {
            let mut command_msg = BytesMut::with_capacity(6);
            command_msg.put_bytes(0xff, 2);
            command_msg.put_u16(0x0002);
            command_msg.put_u16(0x0001);
            command_msg.put_u8(cmd);
            client.write_all(&command_msg).await.unwrap();

            let bytes = sensor.read().await.unwrap();
            let response = sensor.process(&bytes).await;
            assert!(response.is_ok());
            assert_eq!(response.unwrap(), 0x01) // unknown command
        }
    }
}
