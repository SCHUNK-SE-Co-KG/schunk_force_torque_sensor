use crate::output_rate::{OutputRateMode, OutputRateState};
use crate::udp_destination_port::UdpDestinationPortState;
use bytes::{BufMut, BytesMut};
use std::net::SocketAddr;
use std::time::Instant;
use tokio::net::UdpSocket;

pub async fn stream_ft_data(
    output_rate: OutputRateState,
    udp_destination_port: UdpDestinationPortState,
) {
    let socket = UdpSocket::bind("0.0.0.0:0").await.unwrap();

    let sync: u16 = 0xFFFF;
    let mut counter: u16 = 0;

    let mut packet_id: u8 = 0;
    let mut next_time = Instant::now();

    let start_time = Instant::now();
    let f = 1.0; // Hz

    loop {
        let mode = output_rate.get();
        let interval = mode.packet_interval();
        let mut buf = BytesMut::with_capacity(6 + payload_len(mode) as usize);

        let elapsed = start_time.elapsed().as_secs_f32();

        // header
        buf.put_u16_le(sync);
        buf.put_u16_le(counter);
        buf.put_u16_le(payload_len(mode));

        // payload
        buf.put_u8(packet_id);
        put_samples(&mut buf, mode, elapsed, f);

        let target: SocketAddr = format!(
            "127.0.0.1:{}",
            udp_destination_port.get()
        )
        .parse()
        .unwrap();
        let _ = socket.send_to(&buf, &target).await;
        packet_id = packet_id.wrapping_add(1);

        while Instant::now() < next_time {}
        next_time += interval;
        counter = counter.wrapping_add(1);
    }
}

fn payload_len(mode: OutputRateMode) -> u16 {
    (1 + mode.samples_per_packet * 28) as u16
}

fn put_samples(buf: &mut BytesMut, mode: OutputRateMode, elapsed: f32, f: f32) {
    for sample_index in 0..mode.samples_per_packet {
        let sample_elapsed = elapsed + mode.sample_period.as_secs_f32() * sample_index as f32;
        let omega = 2.0 * std::f32::consts::PI * f;

        let fx = (omega * sample_elapsed).sin();
        let fy = (omega * sample_elapsed + 1.0).sin();
        let fz = (omega * sample_elapsed + 2.0).sin();
        let tx = (omega * sample_elapsed + 3.0).sin();
        let ty = (omega * sample_elapsed + 4.0).sin();
        let tz = (omega * sample_elapsed + 5.0).sin();

        let status_bits: i32 = 0x00000001;

        buf.put_i32_le(status_bits);
        buf.put_f32_le(fx);
        buf.put_f32_le(fy);
        buf.put_f32_le(fz);
        buf.put_f32_le(tx);
        buf.put_f32_le(ty);
        buf.put_f32_le(tz);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn test_single_sample_payload_length_matches_sensor() {
        let mode = OutputRateMode::from_enum(0).unwrap();

        assert_eq!(payload_len(mode), 29);
    }

    #[test]
    fn test_packaged_payload_length_matches_sensor() {
        let mode = OutputRateMode::from_enum(10).unwrap();

        assert_eq!(payload_len(mode), 449);
    }

    #[test]
    fn test_packaged_mode_metadata_matches_sensor() {
        let mode = OutputRateMode::from_enum(10).unwrap();

        assert_eq!(mode.requested_hz, 8000);
        assert_eq!(mode.udp_packet_rate_hz, 500);
        assert_eq!(mode.samples_per_packet, 16);
        assert_eq!(mode.packet_interval(), Duration::from_micros(2000));
        assert_eq!(mode.sample_period, Duration::from_micros(125));
    }

    #[test]
    fn test_packaged_payload_contains_one_packet_id_and_sixteen_samples() {
        let mode = OutputRateMode::from_enum(10).unwrap();
        let mut buf = BytesMut::with_capacity(6 + payload_len(mode) as usize);

        buf.put_u16_le(0xffff);
        buf.put_u16_le(42);
        buf.put_u16_le(payload_len(mode));
        buf.put_u8(7);
        put_samples(&mut buf, mode, 0.0, 1.0);

        assert_eq!(buf.len(), 455);
        assert_eq!(u16::from_le_bytes([buf[4], buf[5]]), 449);
        assert_eq!(buf[6], 7);
        for sample_index in 0..16 {
            let sample_start = 7 + sample_index * 28;
            assert_eq!(
                i32::from_le_bytes([
                    buf[sample_start],
                    buf[sample_start + 1],
                    buf[sample_start + 2],
                    buf[sample_start + 3]
                ]),
                1
            );
        }
    }
}
