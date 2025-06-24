use crate::sensor::Sensor;
use bytes::{BufMut, BytesMut};
use tokio::io;
use tokio::net::TcpListener;

pub async fn handle_requests() -> io::Result<()> {
    let listener = TcpListener::bind("127.0.0.1:8082").await?;

    loop {
        let (socket, _) = listener.accept().await?;
        tokio::spawn(async move {
            let mut sensor = Sensor::new(socket);
            match sensor.read().await {
                Ok(msg) => {
                    // Handle connections without data exchange
                    if msg.len() < 6 {
                        return;
                    }

                    // Process data
                    let payload = sensor.process(&msg).await.unwrap();

                    // Response msg
                    let counter = 0x0001;
                    let payload_len = payload.to_vec().len();
                    let mut response = BytesMut::new();
                    response.put_bytes(0xff, 2);
                    response.put_u16_le(counter);
                    response.put_u16_le(payload_len as u16);
                    response.put_slice(&payload);

                    // Send back
                    let _ = sensor.write(&response).await;
                }
                Err(e) => eprintln!("Error reading from socket: {}", e),
            }
        });
    }
}
