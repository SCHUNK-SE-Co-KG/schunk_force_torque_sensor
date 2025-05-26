mod sensor;

use sensor::Sensor;
use tokio::io;
use tokio::net::TcpListener;

#[tokio::main]
async fn main() -> io::Result<()> {
    let listener = TcpListener::bind("127.0.0.1:6000").await?;

    loop {
        let (socket, _) = listener.accept().await?;
        tokio::spawn(async move {
            let mut sensor = Sensor::new(socket);
            match sensor.read().await {
                Ok(message) => {
                    println!("Received: {}", message);
                    let _ = sensor.write(b"Response from server").await;
                }
                Err(e) => eprintln!("Error reading from socket: {}", e),
            }
        });
    }
}
