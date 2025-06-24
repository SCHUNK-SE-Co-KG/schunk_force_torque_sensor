mod sensor;
mod tcp;
mod udp;
use tokio::io;

#[tokio::main]
async fn main() -> io::Result<()> {
    tokio::spawn(async {
        udp::stream_ft_data().await;
    });

    tcp::handle_requests().await
}
