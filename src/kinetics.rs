struct DeltaMovement {
    last_location: isize,
    last_speed: isize,
    last_acceleration: isize
}

impl DeltaMovement {
    fn default(last_location: isize) -> Self {
        Self { last_location, last_speed: 0, last_acceleration: 0 }
    }
    fn update_location(&mut self, location: isize) -> ( isize, isize, isize, isize) {
        let delta_t = elapsed();
        let speed = (location - self.last_location) / delta_t;
        let acceleration = (speed - self.last_speed) / delta_t;
        let jerk = (acceleration - self.last_acceleration) / delta_t;
        self.last_location = location;
        self.last_speed = speed;
        self.last_acceleration = acceleration;
        (location, speed, acceleration, jerk)
    }
}
