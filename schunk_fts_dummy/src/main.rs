mod output_rate;
mod sensor;
mod tcp;
mod udp;
mod udp_destination_port;
use std::env;
use tokio::io;

const DEFAULT_TCP_HOST: &str = "127.0.0.1";
const DEFAULT_TCP_PORT: u16 = 8082;
const DEFAULT_UDP_TARGET_HOST: &str = "127.0.0.1";

fn tcp_bind_addr() -> String {
    if let Ok(addr) = env::var("SCHUNK_FTS_DUMMY_TCP_ADDR") {
        return addr;
    }

    let host =
        env::var("SCHUNK_FTS_DUMMY_TCP_HOST").unwrap_or_else(|_| DEFAULT_TCP_HOST.to_string());
    let port = env::var("SCHUNK_FTS_DUMMY_TCP_PORT")
        .ok()
        .and_then(|port| port.parse::<u16>().ok())
        .unwrap_or(DEFAULT_TCP_PORT);
    format!("{host}:{port}")
}

fn udp_target_host() -> String {
    env::var("SCHUNK_FTS_DUMMY_UDP_TARGET_HOST")
        .unwrap_or_else(|_| DEFAULT_UDP_TARGET_HOST.to_string())
}

#[tokio::main]
async fn main() -> io::Result<()> {
    let output_rate = output_rate::OutputRateState::default();
    let udp_output_rate = output_rate.clone();
    let udp_destination_port = udp_destination_port::UdpDestinationPortState::default();
    let udp_destination_port_stream = udp_destination_port.clone();
    let tcp_bind_addr = tcp_bind_addr();
    let udp_target_host = udp_target_host();

    tokio::spawn(async move {
        udp::stream_ft_data(
            udp_output_rate,
            udp_destination_port_stream,
            udp_target_host,
        )
        .await;
    });

    tcp::handle_requests(output_rate, udp_destination_port, &tcp_bind_addr).await
}
