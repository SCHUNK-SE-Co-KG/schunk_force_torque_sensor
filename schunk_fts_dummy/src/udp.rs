use bytes::{BufMut, BytesMut};
use std::net::SocketAddr;
use std::time::{Duration, Instant};
use tokio::net::UdpSocket;

pub async fn stream_ft_data() {
    let socket = UdpSocket::bind("0.0.0.0:0").await.unwrap();
    let target: SocketAddr = "127.0.0.1:54843".parse().unwrap();

    let sync: u16 = 0xFFFF;
    let mut counter: u16 = 0;
    let payload_len: u16 = 29;

    let mut packet_id: u8 = 0;

    let frequency_hz = 1000.0;
    let interval = Duration::from_secs_f64(1.0 / frequency_hz);
    let mut next_time = Instant::now();

    let start_time = Instant::now();
    let f = 1.0; // Hz

    loop {
        let mut buf = BytesMut::with_capacity(6 + payload_len as usize);

        let elapsed = start_time.elapsed().as_secs_f32();
        let omega = 2.0 * std::f32::consts::PI * f;

        let fx = (omega * elapsed).sin();
        let fy = (omega * elapsed + 1.0).sin(); // phase shift for variety
        let fz = (omega * elapsed + 2.0).sin();
        let tx = (omega * elapsed + 3.0).sin();
        let ty = (omega * elapsed + 4.0).sin();
        let tz = (omega * elapsed + 5.0).sin();

        let status_bits: i32 = 0x00000001; // assume sensor is ready for operation

        // header
        buf.put_u16_le(sync);
        buf.put_u16_le(counter);
        buf.put_u16_le(payload_len);

        // payload
        buf.put_u8(packet_id);
        buf.put_i32_le(status_bits);
        buf.put_f32_le(fx);
        buf.put_f32_le(fy);
        buf.put_f32_le(fz);
        buf.put_f32_le(tx);
        buf.put_f32_le(ty);
        buf.put_f32_le(tz);

        let _ = socket.send_to(&buf, &target).await;
        packet_id = packet_id.wrapping_add(1);

        while Instant::now() < next_time {}
        next_time += interval;
        counter = counter.wrapping_add(1);
    }
}
