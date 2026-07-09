use crate::output_rate::OutputRateState;
use crate::sensor::Sensor;
use crate::udp_destination_port::UdpDestinationPortState;
use bytes::{BufMut, BytesMut};
use tokio::io;
use tokio::net::TcpListener;

pub async fn handle_requests(
    output_rate: OutputRateState,
    udp_destination_port: UdpDestinationPortState,
    bind_addr: &str,
) -> io::Result<()> {
    let listener = TcpListener::bind(bind_addr).await?;

    loop {
        let (socket, addr) = listener.accept().await?;
        let output_rate = output_rate.clone();
        let udp_destination_port = udp_destination_port.clone();
        tokio::spawn(async move {
            let mut sensor =
                Sensor::with_state(socket, output_rate, udp_destination_port);
            let mut counter: u16 = 1;
            loop {
                match sensor.read().await {
                    Ok(msg) => {
                        // Handle connections without data exchange
                        if msg.len() < 6 {
                            eprintln!("Received incomplete message");
                            return;
                        }

                        // Process data
                        eprintln!("Received message: {:?}", msg);
                        match sensor.process(&msg).await {
                            Ok(payload) => {
                                let payload_len = payload.len() as u16;

                                // Response msg
                                let mut response = BytesMut::new();
                                response.put_bytes(0xff, 2);
                                response.put_u16_le(counter);
                                response.put_u16_le(payload_len as u16);
                                response.put_slice(&payload);

                                if let Err(e) = sensor.write(&response).await {
                                    eprintln!("Write error to {}: {}", addr, e);
                                    break; // close socket on write failure
                                }

                                counter = counter.wrapping_add(1);
                            }
                            Err(e) => {
                                eprintln!("Processing error for {}: {}", addr, e);
                                break;
                            }
                        }
                    }
                    Err(e) => {
                        eprintln!("Socket read error from {}: {}", addr, e);
                        break;
                    }
                }
            }
            eprintln!("Connection with {} closed", addr);
        });
    }
}
