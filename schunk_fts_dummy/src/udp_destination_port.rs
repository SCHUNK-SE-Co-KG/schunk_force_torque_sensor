use std::sync::{Arc, Mutex};

pub const DEFAULT_UDP_DESTINATION_PORT: u16 = 54843;
pub const MIN_UDP_DESTINATION_PORT: u16 = 1;
pub const MAX_UDP_DESTINATION_PORT: u16 = 65534;

#[derive(Clone, Debug)]
pub struct UdpDestinationPortState {
    port: Arc<Mutex<u16>>,
}

impl UdpDestinationPortState {
    pub fn new() -> Self {
        Self {
            port: Arc::new(Mutex::new(DEFAULT_UDP_DESTINATION_PORT)),
        }
    }

    pub fn get(&self) -> u16 {
        *self.port.lock().unwrap()
    }

    pub fn set(&self, port: u16) -> bool {
        if !(MIN_UDP_DESTINATION_PORT..=MAX_UDP_DESTINATION_PORT).contains(&port) {
            return false;
        }
        *self.port.lock().unwrap() = port;
        true
    }
}

impl Default for UdpDestinationPortState {
    fn default() -> Self {
        Self::new()
    }
}
