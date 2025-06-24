use bytes::{BufMut, BytesMut};
use std::net::SocketAddr;
use tokio::net::UdpSocket;

pub async fn stream_ft_data() {
    let socket = UdpSocket::bind("0.0.0.0:0").await.unwrap();
    let target: SocketAddr = "127.0.0.1:54843".parse().unwrap();

    let mut packet_id: u8 = 0;

    loop {
        let mut buf = BytesMut::with_capacity(29);

        let status_bits: i32 = 0x00000000;
        let fx: f32 = 1.0;
        let fy: f32 = 2.0;
        let fz: f32 = 3.0;
        let tx: f32 = 4.0;
        let ty: f32 = 5.0;
        let tz: f32 = 6.0;

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
    }
}
