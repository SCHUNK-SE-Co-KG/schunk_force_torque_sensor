mod sensor;

use bytes::{BufMut, BytesMut};
use sensor::Sensor;
use tokio::io;
use tokio::net::TcpListener;

#[tokio::main]
async fn main() -> io::Result<()> {
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

                    // Process commands
                    let cmd_index = 6;
                    let cmd_id = msg[cmd_index];
                    let error_code = sensor.process(&msg).await.unwrap();

                    let mut response = BytesMut::new();
                    let counter = 0x0001;
                    let payload_len = 0x0002;
                    response.put_bytes(0xff, 2);
                    response.put_u16(counter);
                    response.put_u16(payload_len);
                    response.put_u8(cmd_id);
                    response.put_u8(error_code);
                    let _ = sensor.write(&response).await;
                }
                Err(e) => eprintln!("Error reading from socket: {}", e),
            }
        });
    }
}
