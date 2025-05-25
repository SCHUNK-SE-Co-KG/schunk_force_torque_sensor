pub struct Sensor {}

impl Sensor {
    pub(crate) fn new() -> Self {
        Self {}
    }
}

// Tests -----------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sensor_initializes_as_expected() {
        let _sensor = Sensor::new();
    }
}
