use tokio::io::{AsyncRead, AsyncWrite};

pub struct Sensor<T> {
    stream: T,
}

impl<T> Sensor<T>
where
    T: AsyncRead + AsyncWrite + Unpin + Send + 'static,
{
    pub fn new(stream: T) -> Self {
        Self { stream }
    }

    pub async fn read(&mut self) -> tokio::io::Result<String> {
        use tokio::io::AsyncReadExt;
        let mut buf = [0; 1024];
        let n = self.stream.read(&mut buf).await?;
        Ok(String::from_utf8_lossy(&buf[..n]).to_string())
    }

    pub async fn write(&mut self, msg: &[u8]) -> tokio::io::Result<()> {
        use tokio::io::AsyncWriteExt;
        self.stream.write_all(msg).await
    }
}

// Tests -----------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use tokio::io::{AsyncReadExt, AsyncWriteExt, duplex};

    #[tokio::test]
    async fn test_read() {
        let (mut client, server) = duplex(1024);
        let mut sensor = Sensor::new(server);

        tokio::spawn(async move {
            let _ = client.write_all(b"test message").await;
        });

        let msg = sensor.read().await.unwrap();
        assert_eq!(msg, "test message");
    }

    #[tokio::test]
    async fn test_write() {
        let (client, mut server) = duplex(1024);
        let mut sensor = Sensor::new(client);

        sensor.write(b"Hello back").await.unwrap();

        let mut buf = [0; 1024];
        let n = server.read(&mut buf).await.unwrap();

        let received = &buf[..n];
        assert_eq!(received, b"Hello back");
    }
}
