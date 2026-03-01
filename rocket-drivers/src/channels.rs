use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

/// Module to contain all global asynchronous channels.
/// This is to keep track of them all and present a single API for tasks.

/// Wifi Channels
use crate::wifi::LedState;
/// Global asynchronous channel for passing LED commands to the wifi_task.
/// Capacity is 2 messages to prevent blocking callers during heavy SPI activity.
pub static LED_CHANNEL: Channel<CriticalSectionRawMutex, LedState, 2> = Channel::new();
