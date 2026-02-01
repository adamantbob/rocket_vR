use core::cell::Cell;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

/// A generic thread-safe container for Blackboard-style data sharing.
pub struct DataCell<T: Copy + Default> {
    storage: Mutex<CriticalSectionRawMutex, Cell<T>>,
}

impl<T: Copy + Default> DataCell<T> {
    /// Create a new cell with an initial value.
    pub const fn new(init: T) -> Self {
        Self {
            storage: Mutex::new(Cell::new(init)),
        }
    }

    /// Update the data in the cell (The "Write").
    pub fn update(&self, data: T) {
        self.storage.lock(|cell| {
            cell.set(data);
        });
    }

    /// Fetch the latest data from the cell (The "Read").
    pub fn read(&self) -> T {
        self.storage.lock(|cell| cell.get())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_datacell_init() {
        let cell = DataCell::new(42);
        assert_eq!(cell.read(), 42);
    }

    #[test]
    fn test_datacell_update() {
        let cell = DataCell::new(0);
        cell.update(100);
        assert_eq!(cell.read(), 100);
    }

    #[test]
    fn test_datacell_default() {
        #[derive(Copy, Clone, Default, PartialEq, Debug)]
        struct MyData {
            a: i32,
            b: f32,
        }
        let cell = DataCell::new(MyData::default());
        assert_eq!(cell.read(), MyData { a: 0, b: 0.0 });
    }
}
