mod output_rate;
mod sensor;
mod tcp;
mod udp_destination_port;
mod udp;
use tokio::io;

#[tokio::main]
async fn main() -> io::Result<()> {
    let output_rate = output_rate::OutputRateState::default();
    let udp_output_rate = output_rate.clone();
    let udp_destination_port = udp_destination_port::UdpDestinationPortState::default();
    let udp_destination_port_stream = udp_destination_port.clone();

    tokio::spawn(async move {
        udp::stream_ft_data(udp_output_rate, udp_destination_port_stream).await;
    });

    tcp::handle_requests(output_rate, udp_destination_port).await
}
